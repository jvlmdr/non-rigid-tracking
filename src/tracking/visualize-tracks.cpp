#include "tracking/using.hpp"
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "tracking/track-list.hpp"
#include "tracking/translation-warp.hpp"
#include "util/random-color.hpp"
#include <boost/format.hpp>

using namespace tracking;

DEFINE_bool(display, true, "Show tracking in window");
DEFINE_string(save, "", "Directory to save frames to, ignored if empty");

DEFINE_int32(radius, 8, "Half of [patch size - 1]");

// Visualization parameters.
const double SATURATION = 0.99;
const double BRIGHTNESS = 0.99;
const int LINE_THICKNESS = 1;

string makeFilename(const string& format, int n) {
  return boost::str(boost::format(format) % n);
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << argv[0] << " tracks video";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

void visualizeTracks(const TrackList& tracks,
                     cv::VideoCapture& capture,
                     int radius,
                     bool display,
                     const string& save) {
  typedef map<int, cv::Vec3b> ColorMap;
  ColorMap colors;

  // Draw tracks?
  bool render = (display || !save.empty());

  // Loop state.
  bool end = false;
  int n = 0;

  typedef RepeatedPtrField<TrackList::Frame> FrameList;
  const FrameList& frames = tracks.frames();
  FrameList::const_iterator frame = frames.begin();

  // Memory that is re-used every loop.
  cv::Mat color_image;
  cv::Mat image;
  cv::Mat visualization;

  // Read frames of video.
  while (!end) {
    // Read next frame.
    bool ok = capture.read(color_image);
    if (!ok) {
      // Reached end.
      end = true;
      continue;
    }

    CHECK(frame != frames.end());

    if (render) {
      // Convert color to intensity and back again.
      cv::cvtColor(color_image, image, CV_BGR2GRAY);
      cv::cvtColor(image, visualization, CV_GRAY2BGR);
    }

    typedef RepeatedPtrField<TrackList::Point> PointList;
    const PointList& features = frame->points();

    PointList::const_iterator feature;
    for (feature = features.begin(); feature != features.end(); ++feature) {
      // Assign a color to the feature if it doesn't already have one.
      const cv::Vec3b* color;
      ColorMap::const_iterator it = colors.find(feature->id());
      if (it == colors.end()) {
        // Generate a color.
        color = &(colors[feature->id()] = randomColor(SATURATION, BRIGHTNESS));
      } else {
        color = &it->second;
      }

      if (render) {
        // Draw.
        TranslationWarp warp(feature->x(), feature->y());
        cv::Scalar scalar((*color)[0], (*color)[1], (*color)[2]);
        warp.draw(visualization, radius, scalar, LINE_THICKNESS);
      }
    }

    if (display) {
      cv::imshow("Tracks", visualization);
      cv::waitKey(1000. / 30);
    }

    if (!save.empty()) {
      string file = makeFilename(save, n);
      cv::imwrite(file, visualization);
    }

    ++frame;
    n += 1;
  }

  std::cout << "Read " << colors.size() << " tracks" << std::endl;
}

int main(int argc, char** argv) {
  init(argc, argv);

  string tracks_file = argv[1];
  string video_file = argv[2];

  bool ok;

  // Load tracks from file.
  TrackList tracks;
  {
    std::ifstream ifs(tracks_file.c_str(), std::ios::binary);
    if (!ifs) {
      LOG(FATAL) << "Could not open tracks file";
    }
    ok = tracks.ParseFromIstream(&ifs);
    if (!ok) {
      LOG(FATAL) << "Could not parse tracks from file";
    }
  }

  // Open video stream.
  cv::VideoCapture capture;
  ok = capture.open(video_file);
  if (!ok) {
    LOG(FATAL) << "Could not open video stream";
  }

  visualizeTracks(tracks, capture, FLAGS_radius, FLAGS_display, FLAGS_save);

  return 0;
}
