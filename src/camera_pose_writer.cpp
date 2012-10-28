#include "camera_pose_writer.hpp"
#include "world_point_writer.hpp"
#include "matrix_writer.hpp"

CameraPoseWriter::~CameraPoseWriter() {}

void CameraPoseWriter::write(cv::FileStorage& file, const CameraPose& pose) {
  MatrixWriter matrix_writer;
  file << "rotation" << "{";
  cv::Mat rotation(pose.rotation, false);
  matrix_writer.write(file, rotation);
  file << "}";

  WorldPointWriter point_writer;
  file << "center" << "{";
  point_writer.write(file, pose.center);
  file << "}";
}
