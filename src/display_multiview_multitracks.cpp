#include <string>
#include <cstdlib>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "random_color.hpp"
#include "sift_position.hpp"
#include "draw_sift_position.hpp"

#include "multiview_track_list_reader.hpp"
#include "default_reader.hpp"
#include "read_lines.hpp"
#include "iterator_reader.hpp"
#include "sift_position_reader.hpp"
#include "read_image.hpp"

DEFINE_int32(width, 1280, "Screen width");
DEFINE_int32(height, 800, "Screen width");

typedef std::vector<SiftPosition> FeatureSet;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const double LINE_THICKNESS = 2;

struct Collage {
  cv::Size canvas_size;
  cv::Size viewport_size;
  std::vector<cv::Point> offsets;

  cv::Mat viewport(cv::Mat& image, int view) const {
    return image(cv::Rect(offsets[view], viewport_size));
  }
};

std::string makeImageFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
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

bool tooBig(const cv::Size& image, const cv::Size& screen, int m) {
  return image.width / m > screen.width || image.height / m > screen.height;
}

bool initializeCollage(Collage& collage,
                       int& downsample,
                       const cv::Size& screen,
                       int num_views,
                       const std::string& format,
                       const std::string& view,
                       int time) {
  // Load image for the given frame.
  std::string file = makeImageFilename(format, view, time);

  cv::Mat color_image;
  bool ok = readColorImage(file, color_image);
  if (!ok) {
    return false;
  }

  cv::Size image = color_image.size();
  cv::Size canvas(image.width * num_views, image.height);

  downsample = 0;
  int factor = 1;
  while (tooBig(canvas, screen, factor)) {
    downsample += 1;
    factor <<= 1;
  }

  image.width /= factor;
  image.height /= factor;
  canvas = cv::Size(image.width * num_views, image.height);

  collage.viewport_size = image;
  collage.canvas_size = canvas;

  collage.offsets.clear();
  for (int i = 0; i < num_views; i += 1) {
    collage.offsets.push_back(cv::Point(image.width * i, 0));
  }

  return true;
}

bool preloadImages(const Collage& collage,
                   std::vector<cv::Mat>& canvases,
                   const std::string& format,
                   const std::vector<std::string>& views,
                   int num_frames,
                   int downsample) {
  canvases.clear();
  int num_views = views.size();

  for (int t = 0; t < num_frames; t += 1) {
    cv::Mat canvas(collage.canvas_size, cv::DataType<cv::Vec3b>::type);

    for (int i = 0; i < num_views; i += 1) {
      // Load image for this frame.
      std::string file = makeImageFilename(format, views[i], t);
      cv::Mat buffer;
      bool ok = readColorImage(file, buffer);
      if (!ok) {
        return false;
      }

      for (int i = 0; i < downsample; i += 1) {
        cv::pyrDown(buffer, buffer);
      }

      cv::Mat viewport = collage.viewport(canvas, i);
      CHECK(buffer.size() == viewport.size()) << buffer.size().width << "x" <<
          buffer.size().height << " vs " << viewport.size().width << "x" <<
          viewport.size().height;
      CHECK(buffer.type() == viewport.type());

      buffer.copyTo(viewport);
    }

    canvases.push_back(canvas);
  }

  return true;
}

bool loadFrameImages(const Collage& collage,
                     cv::Mat& canvas,
                     const std::string& format,
                     const std::vector<std::string>& views,
                     int time,
                     int downsample) {
  int num_views = views.size();
  canvas.create(collage.canvas_size, cv::DataType<cv::Vec3b>::type);

  for (int i = 0; i < num_views; i += 1) {
    // Load image for this frame.
    std::string file = makeImageFilename(format, views[i], time);
    cv::Mat buffer;
    bool ok = readColorImage(file, buffer);
    if (!ok) {
      return false;
    }

    for (int i = 0; i < downsample; i += 1) {
      cv::pyrDown(buffer, buffer);
    }

    cv::Mat viewport = collage.viewport(canvas, i);
    CHECK(buffer.size() == viewport.size()) << buffer.size().width << "x" <<
        buffer.size().height << " vs " << viewport.size().width << "x" <<
        viewport.size().height;
    CHECK(buffer.type() == viewport.type());

    buffer.copyTo(viewport);
  }

  return true;
}

void drawFeatures(cv::Mat& image, 
                  const FeatureSet& features,
                  const cv::Scalar& color) {
  FeatureSet::const_iterator feature;
  for (feature = features.begin(); feature != features.end(); ++feature) {
    drawSiftPosition(image, *feature, color, LINE_THICKNESS);
  }
}

void drawMultiviewFeatures(const std::map<int, FeatureSet>& views,
                           const Collage& collage,
                           cv::Mat& canvas,
                           const cv::Scalar& color) {
  // Render features in each view.
  std::map<int, FeatureSet>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view) {
    int index = view->first;
    const FeatureSet& features = view->second;
    cv::Mat viewport = collage.viewport(canvas, index);
    drawFeatures(viewport, features, color);
  }
}

int numFeaturesInMultitrack(const MultiviewTrack<FeatureSet>& track) {
  MultiviewTrack<FeatureSet>::ConstFeatureIterator iter(track);
  int n = 0;

  for (iter.begin(); !iter.end(); iter.next()) {
    n += iter.get().second->size();
  }

  return n;
}

bool hasMoreFeatures(const MultiviewTrack<FeatureSet>& lhs,
                     const MultiviewTrack<FeatureSet>& rhs) {
  return numFeaturesInMultitrack(lhs) > numFeaturesInMultitrack(rhs);
}

SiftPosition scaleFeature(const SiftPosition& feature, double scale) {
  return SiftPosition(scale * feature.x, scale * feature.y,
      scale * feature.size, feature.theta);
}

void scaleFeatures(const FeatureSet& unscaled,
                   FeatureSet& scaled,
                   double scale) {
  scaled.clear();
  std::transform(unscaled.begin(), unscaled.end(), std::back_inserter(scaled),
      boost::bind(scaleFeature, _1, scale));
}

void scaleFeatures(const Track<FeatureSet>& unscaled,
                   Track<FeatureSet>& scaled,
                   double scale) {
  scaled.clear();
  Track<FeatureSet>::const_iterator x;
  for (x = unscaled.begin(); x != unscaled.end(); ++x) {
    FeatureSet y;
    scaleFeatures(x->second, y, scale);

    FeatureSet& empty = scaled[x->first];
    empty.swap(y);
  }
}

void scaleFeatures(const MultiviewTrack<FeatureSet>& unscaled,
                   MultiviewTrack<FeatureSet>& scaled,
                   double scale) {
  scaled = MultiviewTrack<FeatureSet>(unscaled.numViews());

  MultiviewTrack<FeatureSet>::const_iterator x;
  MultiviewTrack<FeatureSet>::iterator view = scaled.begin();

  for (x = unscaled.begin(); x != unscaled.end(); ++x) {
    Track<FeatureSet> y;
    scaleFeatures(*x, y, scale);

    view->swap(y);
    ++view;
  }
}

void scaleFeatures(const MultiviewTrackList<FeatureSet>& unscaled,
                   MultiviewTrackList<FeatureSet>& scaled,
                   double scale) {
  scaled = MultiviewTrackList<FeatureSet>(unscaled.numViews());

  MultiviewTrackList<FeatureSet>::const_iterator x;
  for (x = unscaled.begin(); x != unscaled.end(); ++x) {
    MultiviewTrack<FeatureSet> y;
    scaleFeatures(*x, y, scale);

    scaled.push_back(MultiviewTrack<FeatureSet>());
    scaled.back().swap(y);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string tracks_file = argv[1];
  std::string image_format = argv[2];
  std::string views_file = argv[3];
  int num_frames = boost::lexical_cast<int>(argv[4]);
  cv::Size screen_size(FLAGS_width, FLAGS_height);

  bool ok;

  // Load names of views.
  std::vector<std::string> views;
  ok = readLines(views_file, views);
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load tracks.
  MultiviewTrackList<FeatureSet> tracks;
  SiftPositionReader feature_reader;
  VectorReader<SiftPosition> feature_set_reader(feature_reader);
  ok = loadMultiviewTrackList(tracks_file, tracks, feature_set_reader);
  CHECK(ok) << "Could not load tracks";
  LOG(INFO) << "Loaded " << tracks.numTracks() << " multi-view tracks";

  // Sort tracks by number of features.
  std::sort(tracks.begin(), tracks.end(), hasMoreFeatures);

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

  // Initialize canvas.
  int downsample = 0;
  Collage collage;
  ok = initializeCollage(collage, downsample, screen_size, views.size(),
      image_format, views.front(), 0);
  CHECK(ok) << "Could not initialize collage";
  double scale = 1. / std::pow(2., downsample);

  // Scale down features.
  {
    MultiviewTrackList<FeatureSet> scaled;
    scaleFeatures(tracks, scaled, scale);
    tracks.swap(scaled);
  }

  // Load all images.
  LOG(INFO) << "Pre-loading all images...";
  std::vector<cv::Mat> images;
  preloadImages(collage, images, image_format, views, num_frames, downsample);
  LOG(INFO) << "Loaded images";

  // Start at the first track.
  MultiviewTrackList<FeatureSet>::const_iterator track = tracks.begin();
  std::vector<cv::Scalar>::const_iterator color = colors.begin();
  MultiviewTrack<FeatureSet>::TimeIterator frame(*track, 0);
  std::vector<cv::Mat>::const_iterator image = images.begin();
  bool exit = false;
  bool pause = true;
  bool step = false;

  while (!exit) {
    // Get points at this time instant, indexed by feature number.
    std::map<int, FeatureSet> features;
    frame.get(features);

    cv::Mat canvas = image->clone();

    // Visualize all features.
    drawMultiviewFeatures(features, collage, canvas, *color);

    cv::imshow("tracks", canvas);
    char c = cv::waitKey(10);

    if (c == 27) {
      exit = true;
    } else if (c == 'j') {
      if (track != tracks.end()) {
        ++track;
        ++color;
        frame = MultiviewTrack<FeatureSet>::TimeIterator(*track, frame.time());
      }
    } else if (c == 'k') {
      if (track != tracks.begin()) {
        --track;
        --color;
        frame = MultiviewTrack<FeatureSet>::TimeIterator(*track, frame.time());
      }
    } else if (c == '0') {
      // Reset to first frame and pause.
      frame = MultiviewTrack<FeatureSet>::TimeIterator(*track, 0);
      image = images.begin();
      pause = true;
    } else if (c == 'n') {
      // Move to next track, reset to first frame and pause.
      if (track != tracks.end()) {
        ++track;
        ++color;
        frame = MultiviewTrack<FeatureSet>::TimeIterator(*track, 0);
        image = images.begin();
        pause = true;
      }
    } else if (c == 'N') {
      // Move to previous track, reset to first frame and pause.
      if (track != tracks.begin()) {
        --track;
        --color;
        frame = MultiviewTrack<FeatureSet>::TimeIterator(*track, 0);
        image = images.begin();
        pause = true;
      }
    } else if (c == ' ') {
      pause = !pause;
    } else if (c == 'l') {
      // Pause but still step one.
      pause = true;
      step = true;
    }

    if (!exit) {
      if (!pause || step) {
        frame.next();
        ++image;
        if (frame.time() >= num_frames) {
          // Reset to the beginning.
          frame = MultiviewTrack<FeatureSet>::TimeIterator(*track, 0);
          image = images.begin();
        }
        step = false;
      }
    }
  }

  return 0;
}
