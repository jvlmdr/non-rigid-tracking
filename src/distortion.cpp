#include "distortion.hpp"

cv::Point2d undistort(const cv::Point2d& x, double w) {
  double rd = cv::norm(x);
  double ru = std::tan(rd * w) / (2. * std::tan(w / 2.));

  return ru / rd * x;
}

cv::Point2d distort(const cv::Point2d& x, double w) {
  double ru = cv::norm(x);
  double rd = 1. / w * std::atan(2. * ru * std::tan(w / 2.));

  return rd / ru * x;
}
