#ifndef SIFT_POSITION_HPP_
#define SIFT_POSITION_HPP_

#include <opencv2/core/core.hpp>

const double SIFT_SIZE_TO_SIGMA = 1.;

struct SiftPosition {
  double x;
  double y;
  double size;
  double theta;

  SiftPosition();
  SiftPosition(double x, double y, double size, double theta);

  cv::Point2d point() const;
};

#endif
