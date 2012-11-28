#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ceres/ceres.h>

#include "read_image.hpp"
#include "warp.hpp"
#include "similarity_warp.hpp"
#include "sift_position.hpp"
#include "flow.hpp"
#include "track_list.hpp"
#include "random_color.hpp"

#include "iterator_reader.hpp"
#include "sift_position_reader.hpp"
#include "sift_position_writer.hpp"
#include "track_list_writer.hpp"
#include "util.hpp"

DEFINE_int32(image_size, 512, "Maximum average dimension of image");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Tracks points in a live demo";
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
}

int main(int argc, char** argv) {
  init(argc, argv);

  cv::VideoCapture capture(0);

  if (!capture.isOpened()) {
    LOG(WARNING) << "Unable to open default camera";
    return 1;
  }

  bool exit = false;

  while (!exit) {
    cv::Mat color_image;
    capture >> color_image;
    int max_pixels = FLAGS_image_size * FLAGS_image_size;
    while (color_image.total() > max_pixels) {
      cv::pyrDown(color_image, color_image);
    }

    cv::Mat gray_image;
    cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);

    cv::imshow("image", gray_image);
    char c = cv::waitKey(1000 / 30);

    if (c == 27) {
      exit = true;
    }
  }

  return 0;
}
