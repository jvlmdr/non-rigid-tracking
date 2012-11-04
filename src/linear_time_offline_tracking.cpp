#include <string>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "track_list.hpp"
#include "track.hpp"

#include "read_image.hpp"

DEFINE_int32(width, 1280, "Screen width");
DEFINE_int32(height, 800, "Screen width");

std::string makeFrameFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Linear time offline tracking" << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

bool tooBig(const cv::Size& image, const cv::Size& screen) {
  return image.width > screen.width || image.height > screen.height;
}

void loadImages(const std::string& format,
                int num_frames,
                const cv::Size& screen,
                std::vector<cv::Mat>& images) {
  images.clear();
  int downsample = 0;

  for (int t = 0; t < num_frames; t += 1) {
    cv::Mat image;
    std::string file = makeFrameFilename(format, t);
    bool ok = readColorImage(file, image);
    CHECK(ok) << "Could not load image";

    if (t == 0) {
      while (tooBig(image.size(), screen)) {
        cv::pyrDown(image, image);
        downsample += 1;
      }
    } else {
      for (int i = 0; i < downsample; i += 1) {
        cv::pyrDown(image, image);
      }
    }

    images.push_back(image);
  }
}

void onMouse(int event, int x, int y, int, void*) {
  if (event == CV_EVENT_LBUTTONDOWN) {
    LOG(INFO) << "Clicked (" << x << ", " << y << ")";
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int num_frames = boost::lexical_cast<int>(argv[2]);
  CHECK(num_frames > 0) << "Need at least one frame, two would be great";
  cv::Size screen_size(FLAGS_width, FLAGS_height);

  // Pre-load images.
  LOG(INFO) << "Loading images...";
  std::vector<cv::Mat> images;
  loadImages(image_format, num_frames, screen_size, images);
  LOG(INFO) << "Loaded " << images.size() << " images";

  TrackList<cv::Point2d> track;

  int t = 0;
  bool exit = false;
  bool pause = false;

  cv::namedWindow("video");
  cv::setMouseCallback("video", onMouse);

  while (!exit) {
    const cv::Mat& image = images[t];

    cv::imshow("video", image);

    char c = cv::waitKey(30);

    if (c == 27) {
      exit = true;
    } else if (c == ' ') {
      pause = !pause;
    } else if (c == 'j') {
      pause = true;
      t = (t + 1) % num_frames;
    } else if (c == 'k') {
      pause = true;
      t = (t + num_frames - 1) % num_frames;
    }

    if (!exit) {
      if (!pause) {
        t = (t + 1) % num_frames;
      }
    }
  }

  return 0;
}
