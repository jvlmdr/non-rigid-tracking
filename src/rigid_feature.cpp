#include "rigid_feature.hpp"

RigidFeature::RigidFeature() {}

RigidFeature::RigidFeature(double x, double y, double size, double theta)
      : x(x), y(y), size(size), theta(theta) {}

const double* RigidFeature::data() const {
  return &x;
}

double* RigidFeature::data() {
  return &x;
}

