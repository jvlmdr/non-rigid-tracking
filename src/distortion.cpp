#include "distortion.hpp"
#include <glog/logging.h>
#include <cmath>

cv::Point2d distort(const cv::Point2d& x, double w) {
  double r = cv::norm(x);
  return distortRadius(r, w) / r * x;
}

cv::Point2d undistort(const cv::Point2d& y, double w) {
  CHECK(isUndistortable(y, w)) << "This point cannot be undistorted";
  double r = cv::norm(y);
  return undistortRadius(r, w) / r * y;
}

bool isUndistortable(const cv::Point2d& y, double w) {
  return (cv::norm(y) < maxDistortedRadius(w));
}

double distortRadius(double r, double w) {
  return 1. / w * std::atan(2. * r * std::tan(w / 2.));
}

double undistortRadius(double q, double w) {
  return std::tan(q * w) / (2. * std::tan(w / 2.));
}

double maxDistortedRadius(double w) {
  return M_PI / (2. * w);
}

cv::Point2d distortPointAtInfinity(const cv::Point2d& x, double w) {
  double r = cv::norm(x);
  return maxDistortedRadius(w) / r * x;
}
