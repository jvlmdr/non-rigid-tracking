#include "world_point_reader.hpp"

WorldPointReader::~WorldPointReader() {}

bool WorldPointReader::read(const cv::FileNode& node, cv::Point3d& point) {
  if (!::read<double>(node["x"], point.x)) {
    return false;
  }

  if (!::read<double>(node["y"], point.y)) {
    return false;
  }

  if (!::read<double>(node["z"], point.z)) {
    return false;
  }

  return true;
}
