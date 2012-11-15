#include "scale_space_position.hpp"

ScaleSpacePosition::ScaleSpacePosition() : x(0), y(0) {}

ScaleSpacePosition::ScaleSpacePosition(double x, double y, double scale)
    : x(x), y(y), scale(scale) {}

cv::Point2d ScaleSpacePosition::point() const {
  return cv::Point2d(x, y);
}
