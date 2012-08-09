#include "camera_pose_reader.hpp"
#include "world_point_reader.hpp"

CameraPoseReader::~CameraPoseReader() {}

void CameraPoseReader::read(const cv::FileNode& node, CameraPose& pose) {
  cv::Mat R(pose.rotation, false);
  node["rotation"] >> R;

  WorldPointReader point_reader;
  point_reader.read(node["center"], pose.center);
}
