#include <list>
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
#include "scale_space_position.hpp"
#include "draw_scale_space_position.hpp"

#include "multiview_track_list_reader.hpp"
#include "read_lines.hpp"
#include "scale_space_position_reader.hpp"
#include "read_image.hpp"

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const double LINE_THICKNESS = 2;

DEFINE_int32(radius, 5, "Base radius");

DEFINE_string(output_format, "%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

DEFINE_bool(show_matches, true, "Show cross-view matches?");
DEFINE_bool(exclude_single_view, true, "Show multi-view tracks that aren't?");
DEFINE_bool(show_trails, false, "Show point trails within view?");

struct Collage {
  cv::Mat image;
  cv::Size size;
  std::vector<cv::Point> offsets;

  cv::Mat viewport(int view) {
    return image(cv::Rect(offsets[view], size));
  }
};

std::string makeTimeFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

std::string makeFrameFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

void removeSingleViewTracks(const MultiviewTrackList<ScaleSpacePosition>& input,
                            MultiviewTrackList<ScaleSpacePosition>& output) {
  output = MultiviewTrackList<ScaleSpacePosition>(input.numViews());

  // Copy track list.
  typedef std::list<MultiviewTrack<ScaleSpacePosition> > TrackList;
  TrackList tracks(input.begin(), input.end());

  TrackList::iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Filter out single-view tracks.
    if (track->numViewsPresent() > 1) {
      output.push_back(MultiviewTrack<ScaleSpacePosition>());
      output.back().swap(*track);
    }
  }
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Visualizes multi-view tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks image-format view-names num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 5) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

bool loadImages(const std::string& image_format,
                int time,
                const std::vector<std::string>& views,
                Collage& collage) {
  int num_views = views.size();
  collage.offsets.assign(num_views, cv::Point());

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
      collage.size = image.size();
      collage.image.create(collage.size.height, collage.size.width * num_views,
          cv::DataType<cv::Vec3b>::type);
    }

    collage.offsets[view] = cv::Point(collage.size.width * view, 0);

    // Copy into collage.
    cv::Mat viewport = collage.viewport(view);
    image.copyTo(viewport);
  }

  return true;
}

void drawFeatures(cv::Mat& image,
                  const std::map<int, ScaleSpacePosition>& features,
                  const std::vector<cv::Scalar>& colors) {
  typedef std::map<int, ScaleSpacePosition> FeatureSet;
  typedef std::vector<cv::Scalar> ColorList;

  FeatureSet::const_iterator mapping;
  for (mapping = features.begin(); mapping != features.end(); ++mapping) {
    int index = mapping->first;
    const ScaleSpacePosition& feature = mapping->second;

    drawScaleSpacePosition(image, feature, FLAGS_radius, colors[index],
        LINE_THICKNESS);
  }
}

void getFeaturesInView(
    const std::map<int, std::map<int, ScaleSpacePosition> >& features,
    int view,
    std::map<int, ScaleSpacePosition>& subset) {
  subset.clear();

  // Iterate through multiview observations of each feature.
  std::map<int, std::map<int, ScaleSpacePosition> >::const_iterator observations;
  for (observations = features.begin();
       observations != features.end();
       ++observations) {
    int id = observations->first;

    // Check if this feature appeared in the current view.
    std::map<int, ScaleSpacePosition>::const_iterator observation;
    observation = observations->second.find(view);

    if (observation != observations->second.end()) {
      // If it was, then add it to the list.
      subset[id] = observation->second;
    }
  }
}

void drawFeaturesInAllViews(
    const std::map<int, std::map<int, ScaleSpacePosition> >& features,
    Collage& collage,
    const std::vector<cv::Scalar>& colors) {
  int num_views = collage.offsets.size();

  // Render features in each view.
  for (int view = 0; view < num_views; view += 1) {
    // Access viewport within collage.
    cv::Mat viewport = collage.viewport(view);

    // Get features for this view.
    std::map<int, ScaleSpacePosition> subset;
    getFeaturesInView(features, view, subset);

    drawFeatures(viewport, subset, colors);
  }
}

void drawInterViewMatches(
    const std::map<int, std::map<int, ScaleSpacePosition> >& features,
    Collage& collage,
    const std::vector<cv::Scalar>& colors) {
  std::map<int, std::map<int, ScaleSpacePosition> >::const_iterator views;
  for (views = features.begin(); views != features.end(); ++views) {
    int id = views->first;
    // 
    std::map<int, ScaleSpacePosition>::const_iterator view1;

    for (view1 = views->second.begin(); view1 != views->second.end(); ++view1) {
      std::map<int, ScaleSpacePosition>::const_iterator view2;

      for (view2 = views->second.begin();
           view2 != views->second.end();
           ++view2) {
        // Draw a line connecting view1 and view2.
        cv::Point2d x1(view1->second.x, view1->second.y);
        cv::Point2d x2(view2->second.x, view2->second.y);

        // Offset by position of view.
        x1 += cv::Point2d(collage.offsets[view1->first].x,
                          collage.offsets[view1->first].y);
        x2 += cv::Point2d(collage.offsets[view2->first].x,
                          collage.offsets[view2->first].y);

        // Draw connecting line.
        cv::line(collage.image, x1, x2, colors[id], LINE_THICKNESS);
      }
    }
  }
}

void drawTrail(TrackIterator<ScaleSpacePosition> position,
               cv::Mat& image,
               const cv::Scalar& color,
               int time) {
  // First go looking forwards in time.
}

void drawTrails(const SingleViewTimeIterator<ScaleSpacePosition>& iterator,
                cv::Mat& image,
                const std::vector<cv::Scalar>& colors) {
}

void drawTrailsInAllViews(
    const MultiViewTimeIterator<ScaleSpacePosition>& iterator,
    Collage& collage,
    const std::vector<cv::Scalar>& colors) {
  int num_views = collage.offsets.size();

  // Render features in each view.
  for (int view = 0; view < num_views; view += 1) {
    // Access viewport within collage.
    cv::Mat viewport = collage.viewport(view);

    drawTrails(iterator.view(view), viewport, colors);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  std::string views_file = argv[3];
  int num_frames = boost::lexical_cast<int>(argv[4]);

  bool ok;

  // Load names of views.
  std::vector<std::string> views;
  ok = readLines(views_file, views);
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load tracks.
  MultiviewTrackList<ScaleSpacePosition> tracks;
  ScaleSpacePositionReader feature_reader;
  ok = loadMultiviewTrackList(tracks_file, tracks, feature_reader);
  CHECK(ok) << "Could not load tracks";
  LOG(INFO) << "Loaded " << tracks.numTracks() << " multi-view tracks";

  // Ensure that number of views matches.
  CHECK(num_views == tracks.numViews());
  // Ensure that there aren't more frames than there are in the sequence.
  // TODO: Is there any point storing the number of frames?
  CHECK(tracks.numFrames() <= num_frames);

  // Remove any single-view tracks.
  if (FLAGS_exclude_single_view) {
    MultiviewTrackList<ScaleSpacePosition> multi;
    removeSingleViewTracks(tracks, multi);
    tracks.swap(multi);
  }

  // Generate a color for each track.
  std::vector<cv::Scalar> colors;
  for (int i = 0; i < tracks.numTracks(); i += 1) {
    colors.push_back(randomColor(BRIGHTNESS, SATURATION));
  }

  // Iterate through time.
  MultiViewTimeIterator<ScaleSpacePosition> iterator(tracks);

  for (int time = 0; time < num_frames; time += 1) {
    LOG(INFO) << time;

    // Get points at this time instant, indexed by feature number.
    std::map<int, std::map<int, ScaleSpacePosition> > features;
    iterator.get(features);

    // Load image for each view.
    Collage collage;
    ok = loadImages(image_format, time, views, collage);
    CHECK(ok) << "Could not load images";

    // Visualize all features.
    drawFeaturesInAllViews(features, collage, colors);

    // Visualize inter-view matches.
    if (FLAGS_show_matches) {
      drawInterViewMatches(features, collage, colors);
    }

    // Visualize optical flow trails.
    if (FLAGS_show_trails) {
      drawTrailsInAllViews(iterator, collage, colors);
    }

    if (FLAGS_save) {
      std::string output_file = makeTimeFilename(FLAGS_output_format, time);
      ok = cv::imwrite(output_file, collage.image);
      CHECK(ok) << "Could not save image";
    }

    if (FLAGS_display) {
      cv::imshow("tracks", collage.image);
      cv::waitKey(10);
    }

    iterator.next();
  }

  CHECK(iterator.end()) << "Did not reach end of tracks";

  return 0;
}
