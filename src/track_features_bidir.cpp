#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ceres/ceres.h>

#include "read_image.hpp"
#include "warp.hpp"
#include "similarity_warp.hpp"
#include "sift_position.hpp"
#include "flow.hpp"
#include "track_list.hpp"
#include "random_color.hpp"

#include "iterator_reader.hpp"
#include "sift_position_reader.hpp"
#include "sift_position_writer.hpp"
#include "track_list_writer.hpp"
#include "util.hpp"

DEFINE_int32(max_frames, -1,
    "Maximum number of frames to track in either direction. "
    "Negative for no limit.");
DEFINE_double(min_scale, 1., "Scale below which feature will be dropped");

DEFINE_int32(radius, 8, "Half of [patch size - 1]");

// Scale of Gaussian mask.
// Patch size should be about 2 * (2 or 3 sigma).
const double MASK_SIGMA = 4.;

// Optical flow settings (frame to frame).
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-6;
const double GRADIENT_TOLERANCE = 1e-6;
const double PARAMETER_TOLERANCE = 1e-6;
const bool ITERATION_LIMIT_IS_FATAL = true;
const bool CHECK_CONDITION = true;
const double MAX_CONDITION = 1e3;

// Tracking settings.
// Align to the template from the previous frame?
const bool UPDATE_TEMPLATE = true;
// Maximum average intensity difference as a fraction of the range.
// (A value of 1 means anything is permitted.)
const double MAX_RESIDUAL = 0.05;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

cv::Mat makeGaussian(double sigma, int width) {
  cv::Mat gaussian = cv::Mat_<double>(width, width);
  int radius = (width - 1) / 2;
  double sigma2 = sigma * sigma;

  for (int i = 0; i < width; i += 1) {
    double u = i - radius;
    for (int j = 0; j < width; j += 1) {
      double v = j - radius;
      double d2 = u * u + v * v;
      gaussian.at<double>(i, j) = std::exp(-d2 / (2. * sigma2));
    }
  }

  cv::Mat mask = cv::Mat_<double>::zeros(width, width);
  cv::Point center(radius, radius);
  cv::circle(mask, center, radius, 1., -1);
  cv::multiply(gaussian, mask, gaussian);

  return gaussian;
}

SimilarityWarp constructWarpFromSiftPosition(const SiftPosition& feature,
                                             const SimilarityWarper& warper) {
  // Convert SIFT size to feature scale.
  double sigma = SIFT_SIZE_TO_SIGMA * feature.size;
  double scale = sigma / MASK_SIGMA;

  SimilarityWarp warp(feature.x, feature.y, std::log(scale), feature.theta,
      warper);

  return warp;
}

SiftPosition extractSiftPositionFromWarp(const SimilarityWarp& warp) {
  double scale = std::exp(warp.logScale());
  // Convert SIFT size to feature scale.
  double sigma = scale * MASK_SIGMA;
  double size = sigma / SIFT_SIZE_TO_SIGMA;

  return SiftPosition(warp.x(), warp.y(), size, warp.theta());
}

std::pair<int, SiftPosition> indexedSimilarityWarpToIndexedSiftPosition(
    const std::pair<int, SimilarityWarp>& pair) {
  return std::make_pair(pair.first, extractSiftPositionFromWarp(pair.second));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Tracks keypoints in both directions." << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format frame-number keypoints-file tracks-file" <<
      std::endl;
  usage << std::endl;
  usage << "Parameters:" << std::endl;
  usage << "frame-number -- Frame to start tracking from (zero-indexed)." <<
      std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 5) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

// While tracking a feature, we need...
//   - warp parameters
//   - previous appearance
//   - initial appearance
struct TrackedFeature {
  SimilarityWarp warp;
  cv::Mat previous_patch;
  cv::Mat initial_patch;

  TrackedFeature(const SimilarityWarp& warp, const cv::Mat& patch)
      : warp(warp),
        previous_patch(patch.clone()),
        initial_patch(patch.clone()) {}

  TrackedFeature() : warp(), previous_patch(), initial_patch() {}
};

// Computes the average pixel residual, weighted by W.
double patchResidual(const cv::Mat& A, const cv::Mat& B, const cv::Mat& W) {
  // D = abs(A - B) .* W
  cv::Mat D = cv::abs(A - B);
  cv::multiply(D, W, D);

  double num = std::accumulate(D.begin<double>(), D.end<double>(), 0.);
  double denom = std::accumulate(W.begin<double>(), W.end<double>(), 0.);

  return num / denom;
}

// Tracks a collection of warps through a sequence.
void trackFeatures(const std::vector<SimilarityWarp>& warps,
                   TrackList<SiftPosition>& tracks,
                   int width,
                   const cv::Mat& mask,
                   const std::string& image_format,
                   int time,
                   const FlowOptions& options,
                   int max_duration,
                   bool reverse) {
  int num_keypoints = warps.size();
  tracks = TrackList<SiftPosition>(num_keypoints);

  int duration = 0;
  bool ok = true;
  cv::Mat color_image;
  cv::Mat integer_image;
  cv::Mat image;

  // Set of features currently being tracked.
  typedef std::map<int, TrackedFeature> TrackedFeatureSet;
  TrackedFeatureSet active;

  while (ok && (max_duration < 0 || duration < max_duration)) {
    // Load image.
    std::string file = makeFilename(image_format, time);
    ok = readImage(file, color_image, integer_image);
    if (!ok) {
      // Assume that this function only fails at the end of the sequence.
      continue;
    }

    // Convert to floating point.
    integer_image.convertTo(image, cv::DataType<double>::type, 1. / 255.);

    if (duration == 0) {
      // First frame. Extract a patch for each feature.
      for (int i = 0; i < num_keypoints; i += 1) {
        // Extract patch.
        cv::Mat patch;
        samplePatch(warps[i], image, patch, width, false,
            options.interpolation);

        // Insert into active set.
        TrackedFeature& feature = (active[i] = TrackedFeature());

        // Copy warp parameters.
        feature.warp = warps[i];
        // Copy into previous patch.
        feature.previous_patch = patch.clone();
        // Transfer ownership to set initial patch (using shared pointer).
        std::swap(feature.initial_patch, patch);
      }
    } else {
      // Not first frame. Track!

      // Compute gradients.
      cv::Mat ddx;
      cv::Mat ddy;
      cv::Mat diff = (cv::Mat_<double>(1, 3) << -0.5, 0, 0.5);
      cv::Mat identity = (cv::Mat_<double>(1, 1) << 1);
      cv::sepFilter2D(image, ddx, -1, diff, identity);
      cv::sepFilter2D(image, ddy, -1, identity, diff);

      // Try to track each active feature.
      TrackedFeatureSet::iterator it = active.begin();
      while (it != active.end()) {
        TrackedFeature& feature = it->second;

        const cv::Mat* reference;
        if (UPDATE_TEMPLATE) {
          reference = &feature.previous_patch;
        } else {
          reference = &feature.initial_patch;
        }

        // Track the patch.
        bool tracked = trackPatch(feature.warp, *reference, image, ddx, ddy,
            mask, options);

        if (tracked) {
          // Sample patch for appearance check and/or template update.
          cv::Mat patch;
          samplePatch(feature.warp, image, patch, width, false,
              options.interpolation);

          // Do appearance check.
          double residual = patchResidual(patch, feature.initial_patch, mask);
          if (residual > MAX_RESIDUAL) {
            DLOG(INFO) << "Appearance residual too large (" << residual <<
                " > " << MAX_RESIDUAL << ")";
            tracked = false;
          }

          // Update appearance.
          std::swap(feature.previous_patch, patch);
        }

        if (!tracked) {
          // Remove.
          active.erase(it++);
        } else {
          // Update previous appearance.
          ++it;
        }
      }

      LOG(INFO) << "Tracked " << active.size() << " features into frame " <<
        (duration + 1);
    }

    // Add each feature to the list of tracks.
    TrackedFeatureSet::const_iterator it;
    for (it = active.begin(); it != active.end(); ++it) {
      int id = it->first;
      const TrackedFeature& feature = it->second;

      tracks[id][time] = extractSiftPositionFromWarp(feature.warp);
    }

    if (reverse) {
      time -= 1;
    } else {
      time += 1;
    }
    duration += 1;
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int frame_number = boost::lexical_cast<int>(argv[2]);
  std::string keypoints_file = argv[3];
  std::string tracks_file = argv[4];

  const int DIAMETER = FLAGS_radius * 2 + 1;

  // Load initial keypoints (x, y, size, theta).
  std::vector<SiftPosition> keypoints;
  SiftPositionReader feature_reader;
  bool ok = loadList(keypoints_file, keypoints, feature_reader);
  CHECK(ok) << "Could not load keypoints";

  int num_features = keypoints.size();
  LOG(INFO) << "Loaded " << num_features << " keypoints";

  // Convert each SIFT feature to a tracking feature.
  typedef std::vector<SimilarityWarp> WarpList;
  WarpList warps;

  // Construct similarity warp cost function.
  SimilarityWarper::CostFunction cost_function(new SimilarityWarpFunction());
  SimilarityWarper warper(cost_function, FLAGS_min_scale);

  std::vector<SiftPosition>::const_iterator keypoint;
  for (keypoint = keypoints.begin(); keypoint != keypoints.end(); ++keypoint) {
    // Convert to a warp.
    SimilarityWarp warp = constructWarpFromSiftPosition(*keypoint, warper);
    warps.push_back(warp);
  }

  // Read one image to get size.
  cv::Size size;
  {
    std::string file = makeFilename(image_format, 0);
    cv::Mat color_image;
    cv::Mat gray_image;
    ok = readImage(file, color_image, gray_image);
    CHECK(ok) << "Could not determine image size from first frame";
    size = color_image.size();
  }

  // Linear solver options.
  FlowOptions options;
  options.solver_options.linear_solver_type = ceres::DENSE_QR;
  options.solver_options.max_num_iterations = MAX_NUM_ITERATIONS;
  options.solver_options.function_tolerance = FUNCTION_TOLERANCE;
  options.solver_options.gradient_tolerance = GRADIENT_TOLERANCE;
  options.solver_options.parameter_tolerance = PARAMETER_TOLERANCE;
  options.check_condition = CHECK_CONDITION;
  options.max_condition = MAX_CONDITION;
  options.iteration_limit_is_fatal = ITERATION_LIMIT_IS_FATAL;
  options.interpolation = cv::INTER_AREA;
  // Construct mask.
  cv::Mat mask = makeGaussian(MASK_SIGMA, DIAMETER);

  // Check whether any of the features are initialized as invalid.
  // This warrants a warning.
  WarpList valid;
  for (int i = 0; i < num_features; i += 1) {
    if (warps[i].isValid(size, FLAGS_radius)) {
      valid.push_back(warps[i]);
    }
  }

  int num_valid = valid.size();
  int num_invalid = num_features - num_valid;
  if (num_invalid > 0) {
    LOG(WARNING) << "Some features were invalid to track (" << num_invalid <<
        " / " << num_features << ")";
  }
  //warp_params.swap(valid);
  //num_features = num_valid;

  // Track forwards.
  TrackList<SiftPosition> forward_tracks;
  trackFeatures(warps, forward_tracks, DIAMETER, mask, image_format,
      frame_number, options, FLAGS_max_frames, false);

  // Track backwards.
  TrackList<SiftPosition> reverse_tracks;
  trackFeatures(warps, reverse_tracks, DIAMETER, mask, image_format,
      frame_number, options, FLAGS_max_frames, true);

  // Merge tracks.
  TrackList<SiftPosition> tracks(num_features);
  for (int i = 0; i < num_features; i += 1) {
    Track<SiftPosition> track;
    track.insert(forward_tracks[i].begin(), forward_tracks[i].end());
    track.insert(reverse_tracks[i].begin(), reverse_tracks[i].end());
    tracks[i].swap(track);
  }

  SiftPositionWriter feature_writer;
  ok = saveTrackList(tracks_file, tracks, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
