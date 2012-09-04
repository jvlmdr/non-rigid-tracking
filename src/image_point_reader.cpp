#include "image_point_reader.hpp"

ImagePointReader::~ImagePointReader() {}

bool ImagePointReader::read(const cv::FileNode& node, cv::Point2d& point) {
  if (!::read<double>(node["x"], point.x)) {
    return false;
  }

  if (!::read<double>(node["y"], point.y)) {
    return false;
  }

  return true;
}
