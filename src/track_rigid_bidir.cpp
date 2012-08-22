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
#include "rigid_warp.hpp"
#include "rigid_feature.hpp"
#include "flow.hpp"
#include "track_list.hpp"
#include "random_color.hpp"

#include "vector_reader.hpp"
#include "rigid_feature_reader.hpp"
#include "rigid_feature_writer.hpp"
#include "track_list_writer.hpp"

DEFINE_int32(max_frames, -1,
    "Maximum number of frames to track in either direction. "
    "Negative for no limit.");
DEFINE_bool(display, true, "Show visualization in window.");
DEFINE_bool(save, false, "Save visualization to file.");
DEFINE_string(save_format, "%d.png", "Format for frame filenames.");

// Size (resolution) of window to track.
const int PATCH_RESOLUTION = 9;
const int NUM_PIXELS = PATCH_RESOLUTION * PATCH_RESOLUTION;

// Lucas-Kanade optimization settings.
const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-4;
const double GRADIENT_TOLERANCE = 0;
const double PARAMETER_TOLERANCE = 1e-4;
const bool ITERATION_LIMIT_IS_FATAL = true;
const double MAX_CONDITION = 1e3;

// Do not want to interpolate much below one pixel.
const double MIN_SCALE = 0.5;

// Maximum average intensity difference as a fraction of the range.
// (A value of 1 means anything is permitted.)
const double MAX_AVERAGE_RESIDUAL = 0.1;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

// A static feature has state and color.
struct StaticFeature {
  RigidFeature state;
  cv::Scalar color;

  StaticFeature() : state(), color() {}

  StaticFeature(const RigidFeature& state, const cv::Scalar& color)
      : state(state), color(color) {}
};

typedef std::vector<StaticFeature> FeatureList;

// A tracked feature has state, color and appearance.
struct TrackedFeature {
  RigidFeature state;
  cv::Scalar color;
  cv::Mat appearance;

  TrackedFeature() : state(), color(), appearance() {}

  TrackedFeature(const RigidFeature& state, const cv::Scalar& color)
      : state(state), color(color), appearance() {}

  explicit TrackedFeature(const StaticFeature& feature)
      : state(feature.state), color(feature.color), appearance() {}
};

StaticFeature makeRandomColorFeature(const RigidFeature& feature) {
  return StaticFeature(feature, randomColor(SATURATION, BRIGHTNESS));
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

void trackFeatures(const std::vector<StaticFeature>& features,
                   TrackList<RigidFeature>& tracks,
                   const std::string& image_format,
                   int frame_number,
                   int max_frames,
                   bool reverse,
                   bool display,
                   bool save,
                   const std::string& save_format) {
  int t = frame_number;
  int n = 0;
  bool ok = true;

  RigidWarp warp(PATCH_RESOLUTION);
  WarpTracker tracker(warp, PATCH_RESOLUTION, MAX_NUM_ITERATIONS,
      FUNCTION_TOLERANCE, GRADIENT_TOLERANCE, PARAMETER_TOLERANCE,
      ITERATION_LIMIT_IS_FATAL, MAX_CONDITION);

  // Set of features currently being tracked.
  typedef std::map<int, TrackedFeature> FeatureSet;
  FeatureSet active;

  while (ok && t >= 0 && n < max_frames) {
    // Read image.
    cv::Mat integer_image;
    cv::Mat color_image;
    ok = readImage(makeFilename(image_format, t), color_image, integer_image);

    // Convert to floating point.
    cv::Mat image;
    integer_image.convertTo(image, cv::DataType<double>::type, 1. / 255.);

    tracker.feedImage(image);

    if (n == 0) {
      // First frame.
      // Copy from list into active set.
      int i = 0;
      for (FeatureList::const_iterator feature = features.begin();
           feature != features.end();
           ++feature) {
        // Create new active feature.
        TrackedFeature& active_feature = (active[i] = TrackedFeature(*feature));

        // Sample appearance.
        cv::Mat M = warp.matrix(feature->state.data());
        sampleAffinePatch(image, active_feature.appearance, M, PATCH_RESOLUTION,
            false);

        i += 1;
      }
    } else {
      // Track features from previous frame.
      // Iterate through features and either update or remove each.
      FeatureSet::iterator it = active.begin();

      while (it != active.end()) {
        TrackedFeature& feature = it->second;

        bool remove;
        bool tracked = tracker.track(feature.state.data());

        if (!tracked) {
          // Remove from list.
          remove = true;
          DLOG(INFO) << "Could not track";
        } else {
          // Check that feature did not become too small.
          double scale = feature.state.size / double(PATCH_RESOLUTION);

          if (scale < MIN_SCALE) {
            remove = true;
            DLOG(INFO) << "Scale too small";
          } else {
            // Sample patch at new position in image.
            cv::Mat M = warp.matrix(feature.state.data());
            cv::Mat appearance;
            sampleAffinePatch(image, appearance, M, PATCH_RESOLUTION, false);

            // Check that the feature looks similar.
            double residual = averageResidual(feature.appearance, appearance);

            if (residual > MAX_AVERAGE_RESIDUAL) {
              remove = true;
              DLOG(INFO) << "Residual too large";
            } else {
              // Keep the feature!
              remove = false;

              // Swap the image contents.
              feature.appearance = appearance;
            }
          }
        }

        if (remove) {
          active.erase(it++);
        } else {
          ++it;
        }
      }
    }

    // Add each point to the track.
    for (FeatureSet::const_iterator it = active.begin();
         it != active.end();
         ++it) {
      int i = it->first;
      const TrackedFeature& feature = it->second;
      tracks[i][t] = feature.state;
    }

    // Visualize each point.
    for (FeatureSet::const_iterator it = active.begin();
         it != active.end();
         ++it) {
      const TrackedFeature& feature = it->second;
      warp.draw(color_image, feature.state.data(), PATCH_RESOLUTION, feature.color);
    }

    if (display) {
      cv::imshow("features", color_image);
      cv::waitKey(10);
    }

    if (save) {
      cv::imwrite(makeFilename(save_format, t), color_image);
    }

    LOG(INFO) << "Tracked " << active.size() << " features into frame " << t;

    if (reverse) {
      t -= 1;
    } else {
      t += 1;
    }
    n += 1;
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int frame_number = boost::lexical_cast<int>(argv[2]);
  std::string keypoints_file = argv[3];
  std::string tracks_file = argv[4];

  int max_frames = FLAGS_max_frames;
  if (max_frames < 0) {
    max_frames = std::numeric_limits<int>::max();
  }

  // Load initial keypoints (x, y, scale, theta).
  std::vector<RigidFeature> keypoints;
  RigidFeatureReader feature_reader;
  bool ok = loadList(keypoints_file, keypoints, feature_reader);
  CHECK(ok) << "Could not load keypoints";
  LOG(INFO) << "Loaded " << keypoints.size() << " keypoints";

  // Convert to features.
  FeatureList features;
  std::transform(keypoints.begin(), keypoints.end(),
      std::back_inserter(features), makeRandomColorFeature);

  int num_features = features.size();
  TrackList<RigidFeature> tracks(num_features);

  // Track forwards.
  trackFeatures(features, tracks, image_format, frame_number, max_frames, false,
      FLAGS_display, FLAGS_save, FLAGS_save_format);

  // Track backwards.
  trackFeatures(features, tracks, image_format, frame_number, max_frames, true,
      FLAGS_display, FLAGS_save, FLAGS_save_format);

  RigidFeatureWriter feature_writer;
  ok = saveTrackList(tracks_file, tracks, feature_writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
