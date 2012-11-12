#include "scale_space_position.hpp"

ScaleSpacePosition::ScaleSpacePosition() : x(0), y(0) {}

ScaleSpacePosition::ScaleSpacePosition(double x, double y) : x(x), y(y) {}

cv::Point2d ScaleSpacePosition::point() const {
  return cv::Point2d(x, y);
}
