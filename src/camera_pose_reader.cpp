#include "camera_pose_reader.hpp"
#include "matrix_reader.hpp"
#include "world_point_reader.hpp"

CameraPoseReader::~CameraPoseReader() {}

bool CameraPoseReader::read(const cv::FileNode& node, CameraPose& pose) {
  // Variable-sized matrix header for fixed-size storage.
  cv::Mat R(pose.rotation, false);

  MatrixReader matrix_reader;
  if (!matrix_reader.read(node["rotation"], R)) {
    return false;
  }

  WorldPointReader point_reader;
  if (!point_reader.read(node["center"], pose.center)) {
    return false;
  }

  return true;
}
