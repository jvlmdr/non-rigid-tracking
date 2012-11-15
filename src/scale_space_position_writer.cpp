#include "scale_space_position_writer.hpp"

ScaleSpacePositionWriter::~ScaleSpacePositionWriter() {}

void ScaleSpacePositionWriter::write(cv::FileStorage& file,
                                     const ScaleSpacePosition& position) {
  file << "x" << position.x;
  file << "y" << position.y;
  file << "size" << position.scale;
}
