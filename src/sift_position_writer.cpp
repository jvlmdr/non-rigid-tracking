#include "sift_position_writer.hpp"

SiftPositionWriter::~SiftPositionWriter() {}

void SiftPositionWriter::write(cv::FileStorage& file,
                               const SiftPosition& feature) {
  file << "x" << feature.x;
  file << "y" << feature.y;
  file << "size" << feature.size;
  file << "angle" << feature.theta;
}
