#include "axis_aligned_ellipse.hpp"
#include "util.hpp"

AxisAlignedEllipse::AxisAlignedEllipse() : a(0), b(0), center() {}

AxisAlignedEllipse::AxisAlignedEllipse(double a, double b, cv::Point2d center)
    : a(a), b(b), center(center) {}

bool AxisAlignedEllipse::contains(const cv::Point2d& point) const {
  cv::Point2d d = point - center;
  return sqr(d.x / a) + sqr(d.y / b) <= 1;
}
