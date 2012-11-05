#include "read_image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>

bool readColorImage(const std::string& filename, cv::Mat& image) {
  // Attempt to read next image.
  image = cv::imread(filename, 1);
  if (image.empty()) {
    LOG(WARNING) << "Could not open `" << filename << "'";
    return false;
  }

  return true;
}

bool readImage(const std::string& filename, cv::Mat& color, cv::Mat& gray) {
  if (!readColorImage(filename, color)) {
    return false;
  }

  // Convert to gray.
  cv::cvtColor(color, gray, CV_BGR2GRAY);

  return true;
}

bool readGrayImage(const std::string& filename, cv::Mat& image) {
  cv::Mat color;
  return readImage(filename, color, image);
}
