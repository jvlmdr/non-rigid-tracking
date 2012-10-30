#include "read_image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>

bool readImage(const std::string& filename, cv::Mat& color, cv::Mat& gray) {
  // Attempt to read next image.
  color = cv::imread(filename, 1);
  if (color.empty()) {
    LOG(WARNING) << "Could not open \"" << filename << "\"";
    return false;
  }

  // Convert to gray.
  cvtColor(color, gray, CV_BGR2GRAY);

  return true;
}
