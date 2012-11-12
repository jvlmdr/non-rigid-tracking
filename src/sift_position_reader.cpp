#include "sift_position_reader.hpp"

SiftPositionReader::~SiftPositionReader() {}

bool SiftPositionReader::read(const cv::FileNode& node, SiftPosition& feature) {
  // Check node is not empty.
  if (node.type() == cv::FileNode::NONE) {
    LOG(WARNING) << "Empty file node";
    return false;
  }

  // Check node is a map.
  if (node.type() != cv::FileNode::MAP) {
    LOG(WARNING) << "Expected file node to be a map";
    return false;
  }

  if (!::read<double>(node["x"], feature.x)) {
    LOG(WARNING) << "Could not find `x' parameter";
    return false;
  }

  if (!::read<double>(node["y"], feature.y)) {
    LOG(WARNING) << "Could not find `y' parameter";
    return false;
  }

  if (!::read<double>(node["size"], feature.size)) {
    LOG(WARNING) << "Could not find `size' parameter";
    return false;
  }

  if (!::read<double>(node["angle"], feature.theta)) {
    LOG(WARNING) << "Could not find `angle' parameter";
    return false;
  }

  return true;
}
