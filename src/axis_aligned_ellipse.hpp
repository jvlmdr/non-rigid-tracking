#ifndef AXIS_ALIGNED_ELLIPSE_HPP_
#define AXIS_ALIGNED_ELLIPSE_HPP_

#include <opencv2/core/core.hpp>

struct AxisAlignedEllipse {
  double a;
  double b;
  cv::Point2d center;

  AxisAlignedEllipse();
  AxisAlignedEllipse(double a, double b, cv::Point2d center);

  bool contains(const cv::Point2d& point) const;
};

#endif
