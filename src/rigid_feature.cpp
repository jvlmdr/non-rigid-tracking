#include "rigid_feature.hpp"

RigidFeature::RigidFeature() {}

RigidFeature::RigidFeature(double x, double y, double scale, double theta)
      : x(x), y(y), scale(scale), theta(theta) {}

const double* RigidFeature::data() const {
  return &x;
}

double* RigidFeature::data() {
  return &x;
}

