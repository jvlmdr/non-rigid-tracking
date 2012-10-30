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
#include "iterator_reader.hpp"
#include "sift_position_reader.hpp"
#include "read_image.hpp"

typedef std::vector<SiftPosition> FeatureSet;

const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const double LINE_THICKNESS = 2;

struct Collage {
  cv::Mat image;
  cv::Size size;
  std::vector<cv::Point> offsets;

  cv::Mat viewport(int view) {
    return image(cv::Rect(offsets[view], size));
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

bool initializeCollage(Collage& collage,
                       int num_views,
                       const std::string& format,
                       const std::string& view,
                       int time) {
  // Load image for the given frame.
  std::string file = makeImageFilename(format, view, time);

  cv::Mat image;
  bool ok = readColorImage(file, image);
  if (!ok) {
    return false;
  }

  collage.size = image.size();
  collage.image.create(collage.size.height, collage.size.width * num_views,
      cv::DataType<cv::Vec3b>::type);

  collage.offsets.clear();
  for (int i = 0; i < num_views; i += 1) {
    collage.offsets.push_back(cv::Point(collage.size.width * i, 0));
  }

  return true;
}

bool loadFrameImages(Collage& collage,
                     const std::string& format,
                     const std::vector<std::string>& views,
                     int time) {
  int num_views = views.size();

  for (int i = 0; i < num_views; i += 1) {
    // Load image for this frame.
    std::string file = makeImageFilename(format, views[i], time);
    cv::Mat buffer;
    bool ok = readColorImage(file, buffer);
    if (!ok) {
      return false;
    }

    cv::Mat viewport = collage.viewport(i);
    CHECK(buffer.size() == viewport.size());
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
                           Collage& collage,
                           const cv::Scalar& color) {
  // Render features in each view.
  std::map<int, FeatureSet>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view) {
    int index = view->first;
    const FeatureSet& features = view->second;
    cv::Mat viewport = collage.viewport(index);
    drawFeatures(viewport, features, color);
  }
}

int numFeaturesInMultitrack(const MultiviewTrack<FeatureSet>& track) {
  MultiviewTrack<FeatureSet>::FeatureIterator iter(track);
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

  // Start at the first track.
  MultiviewTrackList<FeatureSet>::const_iterator track = tracks.begin();
  std::vector<cv::Scalar>::const_iterator color = colors.begin();
  MultiviewTrack<FeatureSet>::TimeIterator frame(*track, 0);
  Collage collage;
  bool exit = false;

  ok = initializeCollage(collage, views.size(), image_format, views.front(), 0);
  CHECK(ok) << "Could not initialize collage";

  while (!exit) {
    // Get points at this time instant, indexed by feature number.
    std::map<int, FeatureSet> features;
    frame.get(features);

    // Load image for each view.
    ok = loadFrameImages(collage, image_format, views, frame.time());
    CHECK(ok) << "Could not load images";

    // Visualize all features.
    drawMultiviewFeatures(features, collage, *color);

    cv::imshow("tracks", collage.image);
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
    }

    if (!exit) {
      frame.next();
      if (frame.time() >= num_frames) {
        // Reset to the beginning.
        frame = MultiviewTrack<FeatureSet>::TimeIterator(*track, 0);
      }
    }
  }

  return 0;
}
