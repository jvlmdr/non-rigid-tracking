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
const double LINE_THICKNESS = 2;

DEFINE_string(output_format, "%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

DEFINE_bool(show_matches, false, "Show cross-view matches?");
DEFINE_bool(show_tracks, false, "Show within-view tracks?");

std::string makeTimeFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

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

bool loadImages(const std::string& image_format,
                int time,
                const std::vector<std::string>& views,
                cv::Mat& collage,
                cv::Size& size,
                std::vector<cv::Point>& offsets) {
  int num_views = views.size();
  offsets.assign(num_views, cv::Point());

  for (int view = 0; view < num_views; view += 1) {
    // Load image for this frame.
    cv::Mat image;
    cv::Mat gray_image;
    std::string file = makeFrameFilename(image_format, views[view], time);
    bool ok = readImage(file, image, gray_image);
    if (!ok) {
      return false;
    }

    // Initialize image if uninitialized.
    if (view == 0) {
      size = image.size();
      collage.create(size.height, size.width * num_views,
          cv::DataType<cv::Vec3b>::type);
    }

    offsets[view] = cv::Point(size.width * view, 0);

    // Copy into collage.
    cv::Mat viewport;
    viewport = collage(cv::Rect(offsets[view], size));
    image.copyTo(viewport);
  }

  return true;
}

bool loadFeatures(const MultiviewTrackList<int>& id_tracks,
                  MultiviewTrackList<RigidFeature>& tracks,
                  const std::string& features_format,
                  const std::vector<std::string>& views) {
  // Initialize list of empty tracks.
  int num_features = id_tracks.numTracks();
  int num_views = views.size();
  std::vector<MultiviewTrack<RigidFeature> > track_list;
  track_list.assign(num_features, MultiviewTrack<RigidFeature>(num_views));

  // Iterate through time (to avoid loading all features at once).
  MultiViewTimeIterator<int> iterator(id_tracks);

  while (!iterator.end()) {
    int time = iterator.time();

    for (int view = 0; view < num_views; view += 1) {
      std::vector<RigidFeature> all_features;

      // Load features in this frame.
      std::string file = makeFrameFilename(features_format, views[view], time);
      RigidFeatureReader feature_reader;
      bool ok = loadList(file, all_features, feature_reader);
      if (!ok) {
        return false;
      }

      // Get feature indices matched in this frame.
      std::map<int, int> subset;
      iterator.getView(view, subset);

      // Copy into track.
      std::map<int, int>::const_iterator id;
      for (id = subset.begin(); id != subset.end(); ++id) {
        track_list[id->first].add(Frame(view, time), all_features[id->second]);
      }
    }

    iterator.next();
  }

  tracks.reset(num_views);
  for (int i = 0; i < num_features; i += 1) {
    tracks.add(track_list[i]);
  }

  return true;
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

void getFeaturesInView(
    const std::map<int, std::map<int, RigidFeature> >& features,
    int view,
    std::map<int, RigidFeature>& subset) {
  subset.clear();

  // Iterate through multiview observations of each feature.
  std::map<int, std::map<int, RigidFeature> >::const_iterator observations;
  for (observations = features.begin();
       observations != features.end();
       ++observations) {
    int id = observations->first;

    // Check if this feature appeared in the current view.
    std::map<int, RigidFeature>::const_iterator observation;
    observation = observations->second.find(view);

    if (observation != observations->second.end()) {
      // If it was, then add it to the list.
      subset[id] = observation->second;
    }
  }
}

void drawFeaturesInAllViews(
    const std::map<int, std::map<int, RigidFeature> >& features,
    cv::Mat& collage,
    const cv::Size& size,
    const std::vector<cv::Point>& offsets,
    const std::vector<cv::Scalar>& colors) {
  int num_views = offsets.size();

  // Render features in each view.
  for (int view = 0; view < num_views; view += 1) {
    // Access viewport within collage.
    cv::Mat viewport = collage(cv::Rect(offsets[view], size));

    // Get features for this view.
    std::map<int, RigidFeature> subset;
    getFeaturesInView(features, view, subset);

    drawFeatures(viewport, subset, colors);
  }
}

void drawInterViewMatches(
    const std::map<int, std::map<int, RigidFeature> >& features,
    cv::Mat& collage,
    const cv::Size& size,
    const std::vector<cv::Point>& offsets,
    const std::vector<cv::Scalar>& colors) {
  std::map<int, std::map<int, RigidFeature> >::const_iterator views;
  for (views = features.begin(); views != features.end(); ++views) {
    int id = views->first;
    // 
    std::map<int, RigidFeature>::const_iterator view1;

    for (view1 = views->second.begin(); view1 != views->second.end(); ++view1) {
      std::map<int, RigidFeature>::const_iterator view2;

      for (view2 = views->second.begin();
           view2 != views->second.end();
           ++view2) {
        // Draw a line connecting view1 and view2.
        cv::Point2d x1(view1->second.x, view1->second.y);
        cv::Point2d x2(view2->second.x, view2->second.y);

        // Offset by position of view.
        x1 += cv::Point2d(offsets[view1->first].x, offsets[view1->first].y);
        x2 += cv::Point2d(offsets[view2->first].x, offsets[view2->first].y);

        // Draw connecting line.
        cv::line(collage, x1, x2, colors[id], LINE_THICKNESS);
      }
    }
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string features_format = argv[2];
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
  MultiviewTrackList<int> id_tracks;
  DefaultReader<int> int_reader;
  ok = loadMultiviewTrackList(tracks_file, id_tracks, int_reader);
  CHECK(ok) << "Could not load tracks";
  LOG(INFO) << "Loaded " << id_tracks.numTracks() << " multi-view tracks";

  // Ensure that number of views matches.
  CHECK(num_views == id_tracks.numViews());
  // Ensure that there aren't more frames than there are in the sequence.
  // TODO: Is there any point storing the number of frames?
  CHECK(id_tracks.numFrames() <= num_frames);

  // Load actual features for tracks.
  MultiviewTrackList<RigidFeature> tracks;
  ok = loadFeatures(id_tracks, tracks, features_format, views);
  CHECK(ok) << "Could not load features";

  // Generate a color for each track.
  std::vector<cv::Scalar> colors;
  for (int i = 0; i < tracks.numTracks(); i += 1) {
    colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  }

  // Iterate through time.
  MultiViewTimeIterator<RigidFeature> iterator(tracks);

  for (int time = 0; time < num_frames; time += 1) {
    LOG(INFO) << time;

    // Get points at this time instant, indexed by feature number.
    std::map<int, std::map<int, RigidFeature> > features;
    iterator.get(features);

    // Load image for each view.
    cv::Mat collage;
    cv::Size size;
    std::vector<cv::Point> offsets;
    ok = loadImages(image_format, time, views, collage, size, offsets);
    CHECK(ok) << "Could not load images";

    // Visualize all features.
    drawFeaturesInAllViews(features, collage, size, offsets, colors);

    // Visualize matches 
    if (FLAGS_show_matches) {
      drawInterViewMatches(features, collage, size, offsets, colors);
    }

    if (FLAGS_save) {
      std::string output_file = makeTimeFilename(FLAGS_output_format, time);
      cv::imwrite(output_file, collage);
    }

    if (FLAGS_display) {
      cv::imshow("tracks", collage);
      cv::waitKey(10);
    }

    iterator.next();
  }

  CHECK(iterator.end()) << "Did not reach end of tracks";

  return 0;
}