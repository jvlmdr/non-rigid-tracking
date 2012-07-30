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

WriteRigidFeature::~WriteRigidFeature() {}

void WriteRigidFeature::operator()(cv::FileStorage& file,
                                   const RigidFeature& feature) {
  file << "{:";
  file << "x" << feature.x;
  file << "y" << feature.y;
  file << "size" << feature.size;
  file << "angle" << feature.theta;
  file << "}";
}

ReadRigidFeature::~ReadRigidFeature() {}

void ReadRigidFeature::operator()(const cv::FileNode& node,
                                  RigidFeature& feature) {
  feature.x = static_cast<double>(node["x"]);
  feature.y = static_cast<double>(node["y"]);
  feature.size = static_cast<double>(node["size"]);
  feature.theta = static_cast<double>(node["angle"]);
}
