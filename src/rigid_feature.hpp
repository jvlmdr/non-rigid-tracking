#ifndef RIGID_FEATURE_HPP_
#define RIGID_FEATURE_HPP_

struct RigidFeature {
  double x;
  double y;
  double size;
  double theta;

  RigidFeature();
  RigidFeature(double x, double y, double size, double theta);

  const double* data() const;
  double* data();
};

#endif
