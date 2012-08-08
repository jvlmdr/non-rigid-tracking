#include "image_point_writer.hpp"

ImagePointWriter::~ImagePointWriter() {}

void ImagePointWriter::write(cv::FileStorage& file, const cv::Point2d& point) {
  file << "x" << point.x << "y" << point.y;
}
