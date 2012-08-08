#include "image_point_reader.hpp"

ImagePointReader::~ImagePointReader() {}

void ImagePointReader::read(const cv::FileNode& node, cv::Point2d& point) {
  point.x = static_cast<double>(node["x"]);
  point.y = static_cast<double>(node["y"]);
}
