#include "world_point_writer.hpp"

WorldPointWriter::~WorldPointWriter() {}

void WorldPointWriter::write(cv::FileStorage& file, const cv::Point3d& point) {
  file << "x" << point.x;
  file << "y" << point.y;
  file << "z" << point.z;
}
