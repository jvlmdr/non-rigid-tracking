#ifndef SCALE_SPACE_POSITION_HPP_
#define SCALE_SPACE_POSITION_HPP_

#include <opencv2/core/core.hpp>

struct ScaleSpacePosition {
  double x;
  double y;
  double scale;

  ScaleSpacePosition();
  ScaleSpacePosition(double x, double y);

  cv::Point2d point() const;
};

#endif
