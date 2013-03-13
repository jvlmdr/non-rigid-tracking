#include "videoseg/using.hpp"
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/format.hpp>
#include "videoseg/segmentation.hpp"
#include "videoseg/draw-region.hpp"

using namespace videoseg;

DEFINE_bool(display, true, "Show segmentation in window");
DEFINE_string(save, "", "Directory to save frames to, ignored if empty");

const int FOREGROUND_LABEL = 0;
const int BACKGROUND_LABEL = 1;

string makeFilename(const string& format, int n) {
  return boost::str(boost::format(format) % n);
}

void visualizeSegmentation(const VideoSegmentation& segmentation,
                           cv::VideoCapture& capture,
                           bool display,
                           const string& save) {
  typedef map<int, cv::Vec3b> ColorMap;
  ColorMap colors;

  // Loop state.
  bool end = false;
  int n = 0;
  bool first = true;

  typedef RepeatedPtrField<VideoSegmentation::Frame> FrameList;
  const FrameList& frames = segmentation.frames();
  FrameList::const_iterator frame = frames.begin();

  // Memory that is re-used every loop.
  cv::Mat image;
  cv::Mat visualization;

  // Read frames of video.
  while (!end && frame != frames.end()) {
    // Read next frame.
    bool ok = capture.read(image);
    if (!ok) {
      // Reached end.
      end = true;
      continue;
    }

    visualization.create(image.size(), cv::DataType<cv::Vec3b>::type);

    // Render each region.
    typedef RepeatedPtrField<VideoSegmentation::Frame::Region> RegionList;
    const RegionList& regions = frame->regions();

    RegionList::const_iterator region;
    for (region = regions.begin(); region != regions.end(); ++region) {
      // Function to draw region.
      bool foreground = (region->id() == FOREGROUND_LABEL);

      if (foreground) {
        copyRegion(region->raster(), image, visualization);
      } else {
        fillRegion(region->raster(), visualization, cv::Vec3b(0, 0, 0));
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

  first = false;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << argv[0] << " foreground video";

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  string foreground_file = argv[1];
  string video_file = argv[2];

  bool ok;

  // Load segmentation from file.
  VideoSegmentation segmentation;
  {
    std::ifstream ifs(foreground_file.c_str(), std::ios::binary);
    if (!ifs) {
      LOG(FATAL) << "Could not open segmentation file";
    }
    ok = segmentation.ParseFromIstream(&ifs);
    if (!ok) {
      LOG(FATAL) << "Could not parse segmentation from file";
    }
  }

  // Open video stream.
  cv::VideoCapture capture;
  ok = capture.open(video_file);
  if (!ok) {
    LOG(FATAL) << "Could not open video stream";
  }

  visualizeSegmentation(segmentation, capture, FLAGS_display, FLAGS_save);

  return 0;
}
