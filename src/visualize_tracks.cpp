#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <gflags/gflags.h>

#include "read_image.hpp"
#include "track_list.hpp"
#include "sift_position.hpp"
#include "scale_space_position.hpp"
#include "draw_sift_position.hpp"
#include "random_color.hpp"
#include "feature_drawer.hpp"
#include "sift_feature_drawer.hpp"
#include "translation_feature_drawer.hpp"
#include "scale_space_feature_drawer.hpp"

#include "sift_position_reader.hpp"
#include "scale_space_position_reader.hpp"
#include "track_list_reader.hpp"
#include "image_point_reader.hpp"

// Parameters for random color generation.
const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const int LINE_THICKNESS = 2;

DEFINE_string(output_format, "%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

DEFINE_bool(scale, false, "Use features with scale?");
DEFINE_bool(similarity, false, "Use similarity transform features?");
DEFINE_int32(radius, 5, "Half-width of translation feature window");

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

typedef boost::shared_ptr<FeatureDrawer> DrawerPointer;

void drawFeatures(cv::Mat& image,
                  const std::map<int, DrawerPointer>& features,
                  const std::vector<cv::Scalar>& colors) {
  typedef std::map<int, DrawerPointer> FeatureSet;
  typedef std::vector<cv::Scalar> ColorList;

  FeatureSet::const_iterator mapping;
  for (mapping = features.begin(); mapping != features.end(); ++mapping) {
    int index = mapping->first;
    const DrawerPointer& feature = mapping->second;

    feature->draw(image, colors[index], LINE_THICKNESS);
  }
}

void translationTrackToDrawers(const Track<cv::Point2d>& feature_track,
                               Track<DrawerPointer>& drawer_track,
                               int radius) {
  drawer_track.clear();
  Track<cv::Point2d>::const_iterator feature;
  for (feature = feature_track.begin();
       feature != feature_track.end();
       ++feature) {
    drawer_track[feature->first].reset(
        new TranslationFeatureDrawer(feature->second, radius));
  }
}

void translationTracksToDrawers(const TrackList<cv::Point2d>& feature_tracks,
                                TrackList<DrawerPointer>& drawer_tracks,
                                int radius) {
  drawer_tracks.clear();
  TrackList<cv::Point2d>::const_iterator feature_track;
  for (feature_track = feature_tracks.begin();
       feature_track != feature_tracks.end();
       ++feature_track) {
    Track<DrawerPointer> drawer_track;
    translationTrackToDrawers(*feature_track, drawer_track, radius);

    drawer_tracks.push_back(Track<DrawerPointer>());
    drawer_tracks.back().swap(drawer_track);
  }
}

void siftPositionTrackToDrawers(const Track<SiftPosition>& feature_track,
                                Track<DrawerPointer>& drawer_track) {
  drawer_track.clear();
  Track<SiftPosition>::const_iterator feature;
  for (feature = feature_track.begin();
       feature != feature_track.end();
       ++feature) {
    drawer_track[feature->first].reset(new SiftFeatureDrawer(feature->second));
  }
}

void siftPositionTracksToDrawers(const TrackList<SiftPosition>& feature_tracks,
                                 TrackList<DrawerPointer>& drawer_tracks) {
  drawer_tracks.clear();
  TrackList<SiftPosition>::const_iterator feature_track;
  for (feature_track = feature_tracks.begin();
       feature_track != feature_tracks.end();
       ++feature_track) {
    Track<DrawerPointer> drawer_track;
    siftPositionTrackToDrawers(*feature_track, drawer_track);

    drawer_tracks.push_back(Track<DrawerPointer>());
    drawer_tracks.back().swap(drawer_track);
  }
}

void scaleFeatureTrackToDrawers(const Track<ScaleSpacePosition>& features,
                                Track<DrawerPointer>& drawers,
                                int radius) {
  drawers.clear();

  Track<ScaleSpacePosition>::const_iterator feature;
  for (feature = features.begin(); feature != features.end(); ++feature) {
    drawers[feature->first].reset(
        new ScaleSpaceFeatureDrawer(feature->second, radius));
  }
}

void scaleFeatureTracksToDrawers(
    const TrackList<ScaleSpacePosition>& feature_tracks,
    TrackList<DrawerPointer>& drawer_tracks,
    int radius) {
  drawer_tracks.clear();
  TrackList<ScaleSpacePosition>::const_iterator feature_track;
  for (feature_track = feature_tracks.begin();
       feature_track != feature_tracks.end();
       ++feature_track) {
    Track<DrawerPointer> drawer_track;
    scaleFeatureTrackToDrawers(*feature_track, drawer_track, radius);

    drawer_tracks.push_back(Track<DrawerPointer>());
    drawer_tracks.back().swap(drawer_track);
  }
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes rigid-warp tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks image-format";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
}

int main(int argc, char** argv) {
  init(argc, argv);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  std::string output_format = FLAGS_output_format;

  bool ok;

  TrackList<DrawerPointer> tracks;

  if (FLAGS_similarity) {
    // Load tracks.
    TrackList<SiftPosition> sift_tracks;
    SiftPositionReader feature_reader;
    ok = loadTrackList(tracks_file, sift_tracks, feature_reader);
    CHECK(ok) << "Could not load tracks";

    // Convert SIFT features to generic drawable features.
    siftPositionTracksToDrawers(sift_tracks, tracks);
  } else if (FLAGS_scale) {
    // Load tracks.
    TrackList<ScaleSpacePosition> scale_tracks;
    ScaleSpacePositionReader feature_reader;
    ok = loadTrackList(tracks_file, scale_tracks, feature_reader);
    CHECK(ok) << "Could not load tracks";

    // Convert SIFT features to generic drawable features.
    scaleFeatureTracksToDrawers(scale_tracks, tracks, FLAGS_radius);
  } else {
    // Load tracks.
    TrackList<cv::Point2d> point_tracks;
    ImagePointReader feature_reader;
    ok = loadTrackList(tracks_file, point_tracks, feature_reader);
    CHECK(ok) << "Could not load tracks";

    // Convert SIFT features to generic drawable features.
    translationTracksToDrawers(point_tracks, tracks, FLAGS_radius);
  }

  LOG(INFO) << "Loaded " << tracks.size() << " tracks";

  // Make a list of random colors.
  typedef std::vector<cv::Scalar> ColorList;
  ColorList colors;
  for (int i = 0; i < int(tracks.size()); i += 1) {
    colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  }

  // Iterate through frames in which track was observed.
  TrackListTimeIterator<DrawerPointer> frame(tracks, 0);

  while (!frame.end()) {
    // Get the current time.
    int t = frame.t();

    // Load the image.
    cv::Mat color_image;
    cv::Mat gray_image;
    ok = readImage(makeFilename(image_format, t), color_image, gray_image);
    CHECK(ok) << "Could not read image";

    // Get the features.
    typedef std::map<int, DrawerPointer> FeatureSet;
    FeatureSet features;
    frame.getPoints(features);

    // Draw each one with its color.
    drawFeatures(color_image, features, colors);

    if (FLAGS_save) {
      std::string output_file = makeFilename(output_format, t);
      ok = cv::imwrite(output_file, color_image);
      CHECK(ok) << "Could not save image";
    }

    if (FLAGS_display) {
      cv::imshow("tracks", color_image);
      cv::waitKey(10);
    }

    ++frame;
  }

  return 0;
}
