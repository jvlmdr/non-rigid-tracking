#ifndef SIFT_POSITION_HPP_
#define SIFT_POSITION_HPP_

const double SIFT_SIZE_TO_SIGMA = 1.;

struct SiftPosition {
  double x;
  double y;
  double size;
  double theta;

  SiftPosition();
  SiftPosition(double x, double y, double size, double theta);
};

#endif
