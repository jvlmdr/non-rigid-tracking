#include "scale_space_position_reader.hpp"

ScaleSpacePositionReader::~ScaleSpacePositionReader() {}

bool ScaleSpacePositionReader::read(const cv::FileNode& node,
                                    ScaleSpacePosition& position) {
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

  if (!::read<double>(node["x"], position.x)) {
    LOG(WARNING) << "Could not find `x' parameter";
    return false;
  }

  if (!::read<double>(node["y"], position.y)) {
    LOG(WARNING) << "Could not find `y' parameter";
    return false;
  }

  if (!::read<double>(node["size"], position.scale)) {
    LOG(WARNING) << "Could not find `size' parameter";
    return false;
  }

  return true;

}
