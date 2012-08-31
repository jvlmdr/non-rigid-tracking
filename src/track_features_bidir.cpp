#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
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

#include "vector_reader.hpp"
#include "sift_position_reader.hpp"
#include "sift_position_writer.hpp"
#include "track_list_writer.hpp"
#include "util.hpp"

DEFINE_int32(max_frames, -1,
    "Maximum number of frames to track in either direction. "
    "Negative for no limit.");
DEFINE_bool(display, true, "Show visualization in window.");
DEFINE_bool(save, false, "Save visualization to file.");
DEFINE_string(save_format, "%d.png", "Format for frame filenames.");

// Size (resolution) of window to track.
const int PATCH_SIZE = 17;
// Scale of Gaussian mask.
// Patch size should be about 2 * (2 or 3 sigma).
const double MASK_SIGMA = 4.;
const int NUM_PIXELS = PATCH_SIZE * PATCH_SIZE;

// Optical flow settings (frame to frame).
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-6;
const double GRADIENT_TOLERANCE = 1e-6;
const double PARAMETER_TOLERANCE = 1e-6;
const bool ITERATION_LIMIT_IS_FATAL = true;
const bool CHECK_CONDITION = true;
const double MAX_CONDITION = 1e3;

// Tracking settings.
// Do not want to interpolate much below one pixel.
const double MIN_SCALE = 1.;
// Align to the template from the previous frame?
const bool UPDATE_TEMPLATE = true;
// Maximum average intensity difference as a fraction of the range.
// (A value of 1 means anything is permitted.)
const double MAX_RESIDUAL = 0.05;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

typedef std::vector<double> Params;

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

Params featureToParams(const SiftPosition& feature) {
  Params params(4);

  params[0] = feature.x;
  params[1] = feature.y;
  params[3] = feature.theta;

  // Convert SIFT size to feature scale.
  double sigma = SIFT_SIZE_TO_SIGMA * feature.size;
  double scale = sigma / MASK_SIGMA;
  params[2] = std::log(scale);

  return params;
}

SiftPosition paramsToFeature(const Params& params) {
  SiftPosition feature;

  feature.x = params[0];
  feature.y = params[1];
  feature.theta = params[3];

  // Convert feature scale to SIFT size.
  double scale = std::exp(params[2]);
  double sigma = scale * MASK_SIGMA;
  feature.size = sigma / SIFT_SIZE_TO_SIGMA;

  return feature;
}

std::pair<int, SiftPosition> paramsPairToFeaturePair(
    const std::pair<int, Params>& pair) {
  return std::make_pair(pair.first, paramsToFeature(pair.second));
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
  Params warp_params;
  cv::Mat previous_patch;
  cv::Mat initial_patch;

  TrackedFeature(const Params& params, const cv::Mat& patch)
      : warp_params(params),
        previous_patch(patch.clone()),
        initial_patch(patch.clone()) {}

  TrackedFeature() : warp_params(), previous_patch(), initial_patch() {}
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
void trackFeatures(const Warp& warp,
                   const std::vector<Params>& keypoints,
                   TrackList<Params>& tracks,
                   int width,
                   const cv::Mat& mask,
                   const std::string& image_format,
                   int time,
                   const FlowOptions& options,
                   int max_duration,
                   bool reverse,
                   const WarpValidator* validator) {
  int num_keypoints = keypoints.size();
  tracks = TrackList<Params>(num_keypoints);

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
        samplePatch(warp, &keypoints[i].front(), image, patch, width, false,
            options.interpolation);

        // Insert into active set.
        TrackedFeature& feature = (active[i] = TrackedFeature());

        // Copy warp parameters.
        feature.warp_params = keypoints[i];
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
        bool tracked = trackPatch(warp, *reference, image, ddx, ddy,
            &feature.warp_params.front(), mask, options, validator);

        if (tracked) {
          // Sample patch for appearance check and/or template update.
          cv::Mat patch;
          samplePatch(warp, &feature.warp_params.front(), image, patch, width,
              false, options.interpolation);

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

      tracks[id][time] = feature.warp_params;
    }

    if (reverse) {
      time -= 1;
    } else {
      time += 1;
    }
    duration += 1;
  }
}

class SimilarityWarpValidator : public WarpValidator {
  public:
    SimilarityWarpValidator(const cv::Size& size,
                            int patch_size,
                            double min_scale)
        : size_(size), patch_size_(patch_size), min_scale_(min_scale) {}

    ~SimilarityWarpValidator() {}

    bool check(const double* params) const {
      SimilarityWarpParams p(params[0], params[1], params[2], params[3]);
      double scale = std::exp(p.log_scale);

      if (!isFinite(scale)) {
        DLOG(INFO) << "Scale became infinite";
        return false;
      }

      if (scale < min_scale_) {
        DLOG(INFO) << "Scale too small (" << scale << " < " << min_scale_ <<
            ")";
        return false;
      }

      // Check if region is entirely within image.
      double radius = scale * patch_size_ / 2.;
      // Reduce size by radius of patch.
      int width = size_.width - 2. * radius;
      int height = size_.height - 2. * radius;
      cv::Rect_<double> bounds(radius, radius, width, height);
      if (!bounds.contains(cv::Point2d(p.x, p.y))) {
        DLOG(INFO) << "Feature outside image";
        return false;
      }

      return true;
    }

  private:
    cv::Size size_;
    int patch_size_;
    double min_scale_;
};

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int frame_number = boost::lexical_cast<int>(argv[2]);
  std::string keypoints_file = argv[3];
  std::string tracks_file = argv[4];

  // Load initial keypoints (x, y, size, theta).
  std::vector<SiftPosition> keypoints;
  SiftPositionReader feature_reader;
  bool ok = loadList(keypoints_file, keypoints, feature_reader);
  CHECK(ok) << "Could not load keypoints";

  int num_features = keypoints.size();
  LOG(INFO) << "Loaded " << num_features << " keypoints";

  // Convert each SIFT feature to a tracking feature.
  std::vector<Params> warp_params;
  std::transform(keypoints.begin(), keypoints.end(),
      std::back_inserter(warp_params), featureToParams);

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

  // Track similarity transform.
  SimilarityWarp warp;
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
  cv::Mat mask = makeGaussian(MASK_SIGMA, PATCH_SIZE);
  // Checks that scale is not vanishing or infinite.
  // Use double 
  SimilarityWarpValidator validator(size, PATCH_SIZE, MIN_SCALE);

  // Check whether any of the features are initialized as invalid.
  // This warrants a warning.
  std::vector<Params> valid;
  for (int i = 0; i < num_features; i += 1) {
    if (validator.check(&warp_params[i].front())) {
      valid.push_back(warp_params[i]);
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
  TrackList<Params> forward_tracks;
  trackFeatures(warp, warp_params, forward_tracks, PATCH_SIZE, mask,
      image_format, frame_number, options, FLAGS_max_frames, false, &validator);

  // Track backwards.
  TrackList<Params> reverse_tracks;
  trackFeatures(warp, warp_params, reverse_tracks, PATCH_SIZE, mask,
      image_format, frame_number, options, FLAGS_max_frames, true, &validator);

  // Merge tracks.
  TrackList<Params> param_tracks(num_features);
  for (int i = 0; i < num_features; i += 1) {
    Track<Params> track;
    track.insert(forward_tracks[i].begin(), forward_tracks[i].end());
    track.insert(reverse_tracks[i].begin(), reverse_tracks[i].end());
    param_tracks[i].swap(track);
  }

  // Convert to features.
  TrackList<SiftPosition> tracks(num_features);
  for (int i = 0; i < num_features; i += 1) {
    Track<SiftPosition> track;
    std::transform(param_tracks[i].begin(), param_tracks[i].end(),
        std::inserter(track, track.begin()), paramsPairToFeaturePair);
    tracks[i].swap(track);
  }

  SiftPositionWriter feature_writer;
  ok = saveTrackList(tracks_file, tracks, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
