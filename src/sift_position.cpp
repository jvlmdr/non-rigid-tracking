#include "sift_position.hpp"

SiftPosition::SiftPosition() {}

SiftPosition::SiftPosition(double x, double y, double size, double theta)
      : x(x), y(y), size(size), theta(theta) {}

cv::Point2d SiftPosition::point() const {
  return cv::Point2d(x, y);
}
