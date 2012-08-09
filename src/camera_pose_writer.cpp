#include "camera_pose_writer.hpp"
#include "world_point_writer.hpp"

CameraPoseWriter::~CameraPoseWriter() {}

void CameraPoseWriter::write(cv::FileStorage& file, const CameraPose& pose) {
  file << "rotation" << cv::Mat(pose.rotation, false);

  WorldPointWriter point_writer;
  file << "center" << "{";
  point_writer.write(file, pose.center);
  file << "}";
}
