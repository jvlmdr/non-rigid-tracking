#include "camera_reader.hpp"
#include "camera_pose_reader.hpp"
#include "camera_properties_reader.hpp"

CameraReader::~CameraReader() {}

bool CameraReader::read(const cv::FileNode& node, Camera& camera) {
  CameraProperties intrinsics;
  CameraPropertiesReader properties_reader;
  if (!properties_reader.read(node, intrinsics)) {
    LOG(WARNING) << "Could not load camera intrinsics";
    return false;
  }

  CameraPose extrinsics;
  CameraPoseReader pose_reader;
  if (!pose_reader.read(node, extrinsics)) {
    LOG(WARNING) << "Could not load camera extrinsics";
    return false;
  }

  return true;
}
