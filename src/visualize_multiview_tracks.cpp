#include <string>
#include <cstdlib>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "random_color.hpp"
#include "rigid_feature.hpp"
#include "rigid_warp.hpp"

#include "multiview_track_list_reader.hpp"
#include "default_reader.hpp"
#include "read_lines.hpp"
#include "vector_reader.hpp"
#include "rigid_feature_reader.hpp"
#include "read_image.hpp"

const int PATCH_SIZE = 9;
const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;

DEFINE_string(output_format, "%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

std::string makeFrameFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes multi-view tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks keypoints-format image-format view-names"
      " num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

void drawFeatures(cv::Mat& image,
                  const std::map<int, RigidFeature>& features,
                  const std::vector<cv::Scalar>& colors) {
  typedef std::map<int, RigidFeature> FeatureSet;
  typedef std::vector<cv::Scalar> ColorList;

  RigidWarp warp(PATCH_SIZE);

  FeatureSet::const_iterator mapping;
  for (mapping = features.begin(); mapping != features.end(); ++mapping) {
    int index = mapping->first;
    const RigidFeature& feature = mapping->second;

    warp.draw(image, feature.data(), PATCH_SIZE, colors[index]);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string keypoints_format = argv[2];
  std::string image_format = argv[3];
  std::string views_file = argv[4];
  int num_frames = boost::lexical_cast<int>(argv[5]);

  bool ok;

  // Load names of views.
  std::vector<std::string> views;
  ok = readLines(views_file, views);
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load tracks.
  MultiviewTrackList<int> tracks;
  DefaultReader<int> int_reader;
  ok = loadMultiviewTrackList(tracks_file, tracks, int_reader);
  CHECK(ok) << "Could not load tracks";
  LOG(INFO) << "Loaded " << tracks.numTracks() << " multi-view tracks";

  // Ensure that number of views matches.
  CHECK(num_views == tracks.numViews());
  // Ensure that there aren't more frames than there are in the sequence.
  // TODO: Is there any point storing the number of frames?
  CHECK(tracks.numFrames() <= num_frames);

  // Generate a color for each track.
  std::vector<cv::Scalar> colors;
  for (int i = 0; i < tracks.numTracks(); i += 1) {
    colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  }

  for (int view = 0; view < num_views; view += 1) {
    // Iterate through time for one view.
    SingleViewTimeIterator<int> time_iterator(tracks, view);

    for (int time = 0; time < num_frames; time += 1) {
      // Get a list of points, indexed by feature number.
      std::map<int, int> feature_ids;
      time_iterator.get(feature_ids);

      // Load features for this frame.
      std::vector<RigidFeature> frame_keypoints;
      std::string keypoints_file = makeFrameFilename(keypoints_format,
          views[view], time);
      RigidFeatureReader feature_reader;
      ok = loadList(keypoints_file, frame_keypoints, feature_reader);
      CHECK(ok) << "Could not load keypoints";

      // Load image for this frame.
      cv::Mat image;
      cv::Mat gray_image;
      std::string image_file = makeFrameFilename(image_format, views[view],
          time);
      ok = readImage(image_file, image, gray_image);
      CHECK(ok) << "Could not load image";

      // Look up each keypoint by index.
      std::map<int, RigidFeature> keypoints;
      std::map<int, int>::const_iterator id;
      for (id = feature_ids.begin(); id != feature_ids.end(); ++id) {
        keypoints[id->first] = frame_keypoints[id->second];
      }

      // Visualize.
      drawFeatures(image, keypoints, colors);

      if (FLAGS_save) {
        std::string output_file = makeFrameFilename(FLAGS_output_format,
            views[view], time);
        cv::imwrite(output_file, image);
      }

      if (FLAGS_display) {
        cv::imshow("tracks", image);
        cv::waitKey(10);
      }

      time_iterator.next();
    }

    CHECK(time_iterator.end()) << "Did not reach end of tracks";
  }

  return 0;
}
