#include "sift_position_reader.hpp"

SiftPositionReader::~SiftPositionReader() {}

void SiftPositionReader::read(const cv::FileNode& node, SiftPosition& feature) {
  feature.x = static_cast<double>(node["x"]);
  feature.y = static_cast<double>(node["y"]);
  feature.size = static_cast<double>(node["size"]);
  feature.theta = static_cast<double>(node["angle"]);
}
