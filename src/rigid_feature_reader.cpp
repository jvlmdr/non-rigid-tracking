#include "rigid_feature_reader.hpp"

RigidFeatureReader::~RigidFeatureReader() {}

void RigidFeatureReader::read(const cv::FileNode& node, RigidFeature& feature) {
  feature.x = static_cast<double>(node["x"]);
  feature.y = static_cast<double>(node["y"]);
  feature.size = static_cast<double>(node["size"]);
  feature.theta = static_cast<double>(node["angle"]);
}
