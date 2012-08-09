#include "world_point_reader.hpp"

WorldPointReader::~WorldPointReader() {}

void WorldPointReader::read(const cv::FileNode& node, cv::Point3d& point) {
  point.x = static_cast<double>(node["x"]);
  point.y = static_cast<double>(node["y"]);
  point.z = static_cast<double>(node["z"]);
}
