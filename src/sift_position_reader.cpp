#include "sift_position_reader.hpp"

SiftPositionReader::~SiftPositionReader() {}

bool SiftPositionReader::read(const cv::FileNode& node, SiftPosition& feature) {
  if (!::read<double>(node["x"], feature.x)) {
    return false;
  }

  if (!::read<double>(node["y"], feature.y)) {
    return false;
  }

  if (!::read<double>(node["size"], feature.size)) {
    return false;
  }

  if (!::read<double>(node["angle"], feature.theta)) {
    return false;
  }

  return true;
}
