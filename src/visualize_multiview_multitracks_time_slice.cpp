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
#include "sift_position.hpp"
#include "draw_sift_position.hpp"

#include "multiview_track_list_reader.hpp"
#include "default_reader.hpp"
#include "read_lines.hpp"
#include "vector_reader.hpp"
#include "sift_position_reader.hpp"
#include "read_image.hpp"

typedef std::vector<SiftPosition> FeatureSet;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const double LINE_THICKNESS = 2;

DEFINE_string(output_format, "%d.png", "Location to save image.");
DEFINE_bool(save, false, "Save to file?");
DEFINE_bool(display, true, "Show in window?");

DEFINE_int32(pixels_per_tick, 5, "Pixels per tick on time axis");
DEFINE_int32(radius, 2, "Radius of circles");
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

  Collage() : image(), size(), offsets() {}

  Collage(const Collage& other) 
      : image(other.image.clone()), size(other.size), offsets(other.offsets) {}

  Collage& operator=(const Collage& other) {
    image = other.image.clone();
    size = other.size;
    offsets = other.offsets;

    return *this;
  }
};

std::string makeFrameFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

void removeSingleViewTracks(const MultiviewTrackList<FeatureSet>& input,
                            MultiviewTrackList<FeatureSet>& output) {
  output = MultiviewTrackList<FeatureSet>(input.numViews());

  MultiviewTrackList<FeatureSet>::const_iterator track;
  for (track = input.begin(); track != input.end(); ++track) {
    // Filter out single-view tracks.
    if (track->numViewsPresent() > 1) {
      // Copy track.
      output.push_back(*track);
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

cv::Size getImageSize(const std::string& format,
                      const std::string& view,
                      int time) {
  // Load first image to get size. TODO: Gross!
  cv::Mat image;
  cv::Mat gray;
  std::string file = makeFrameFilename(format, view, time);
  bool ok = readImage(file, image, gray);

  if (!ok) {
    return cv::Size(0, 0);
  } else {
    return image.size();
  }
}

bool loadBackground(Collage& collage,
                    const std::string& format,
                    const std::vector<std::string>& views,
                    int num_frames,
                    int pixels_per_tick) {
  // Clear.
  collage = Collage();

  int num_views = views.size();

  // Get image size from first frame of first view.
  cv::Size image_size = getImageSize(format, views.front(), 0);
  CHECK(image_size.area() > 0) << "Could not determine image size";

  // Take (x, y)-size and make (t, y)-size.
  cv::Size viewport_size(num_frames * pixels_per_tick, image_size.height);
  collage.size = viewport_size;

  // Repeat vertically for multiple views.
  cv::Size collage_size(viewport_size.width, num_views * viewport_size.height);
  collage.image = cv::Mat_<cv::Vec3b>::zeros(collage_size);

  // Set offsets.
  for (int c = 0; c < num_views; c += 1) {
    cv::Point offset(0, c * viewport_size.height);
    collage.offsets.push_back(offset);
  }

  // Load images.
  for (int c = 0; c < num_views; c += 1) {
    cv::Mat viewport = collage.viewport(c);

    for (int t = 0; t < num_frames; t += 1) {
      // Load image.
      cv::Mat image;
      cv::Mat gray_image;
      std::string file = makeFrameFilename(format, views[c], t);
      bool ok = readImage(file, image, gray_image);
      if (!ok) {
        LOG(WARNING) << "Could not load image";
        return false;
      }

      if (image.size() != image_size) {
        LOG(WARNING) << "Image size changed";
        return false;
      }

      // Extract column of pixels.
      for (int i = 0; i < pixels_per_tick; i += 1) {
        cv::Mat dst = viewport.col(t * pixels_per_tick + i);
        image.col(image.rows / 2).copyTo(dst);
      }
    }
  }

  return true;
}

void drawTrack(cv::Mat& image,
               const Track<FeatureSet>& track,
               const cv::Scalar& color,
               int pixels_per_tick,
               int radius) {
  int half = (pixels_per_tick - 1) / 2;

  Track<FeatureSet>::const_iterator point;

  for (point = track.begin(); point != track.end(); ++point) {
    int t = point->first;
    const FeatureSet& features = point->second;

    // Draw each feature.
    FeatureSet::const_iterator feature;

    for (feature = features.begin(); feature != features.end(); ++feature) {
      cv::Point center(t * pixels_per_tick + half, feature->y);
      cv::circle(image, center, radius, color, LINE_THICKNESS);
    }
  }
}

void drawMultiviewTrack(Collage& collage,
                        const MultiviewTrack<FeatureSet>& track,
                        const cv::Scalar& color,
                        int pixels_per_tick,
                        int radius) {
  MultiviewTrack<FeatureSet>::const_iterator view;
  int c = 0;

  for (view = track.begin(); view != track.end(); ++view) {
    cv::Mat viewport = collage.viewport(c);
    drawTrack(viewport, *view, color, pixels_per_tick, radius);
    c += 1;
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  std::string views_file = argv[3];
  int num_frames = boost::lexical_cast<int>(argv[4]);
  int pixels_per_tick = FLAGS_pixels_per_tick;
  int radius = FLAGS_radius;

  bool ok;

  // Load names of views.
  std::vector<std::string> views;
  ok = readLines(views_file, views);
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load a background image.
  Collage background;
  loadBackground(background, image_format, views, num_frames, pixels_per_tick);

  // Load tracks.
  MultiviewTrackList<FeatureSet> tracks;
  SiftPositionReader feature_reader;
  VectorReader<SiftPosition> feature_set_reader(feature_reader);
  ok = loadMultiviewTrackList(tracks_file, tracks, feature_set_reader);
  CHECK(ok) << "Could not load tracks";
  LOG(INFO) << "Loaded " << tracks.numTracks() << " multi-view tracks";

  // Ensure that number of views matches.
  CHECK(num_views == tracks.numViews());
  // Ensure that there aren't more frames than there are in the sequence.
  // TODO: Is there any point storing the number of frames?
  CHECK(tracks.numFrames() <= num_frames);

  // Remove any single-view tracks.
  if (FLAGS_exclude_single_view) {
    MultiviewTrackList<FeatureSet> multi;
    removeSingleViewTracks(tracks, multi);
    tracks.swap(multi);
  }

  // Render tracks through time.
  MultiviewTrackList<FeatureSet>::const_iterator track;
  int i = 0;

  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Copy background.
    Collage collage = background;

    // Pick a random color.
    cv::Scalar color = randomColor(BRIGHTNESS, SATURATION);

    // Draw multi-view track.
    drawMultiviewTrack(collage, *track, color, pixels_per_tick, radius);

    cv::imshow("collage", collage.image);
    cv::waitKey();

    i += 1;
  }

  return 0;
}
