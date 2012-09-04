#include "camera_properties_reader.hpp"

CameraPropertiesReader::~CameraPropertiesReader() {}

bool CameraPropertiesReader::read(const cv::FileNode& node,
                                  CameraProperties& camera) {
  if (!::read<int>(node["ImageWidth"], camera.image_size.width)) {
    return false;
  }

  if (!::read<int>(node["ImageHeight"], camera.image_size.height)) {
    return false;
  }

  if (!::read<double>(node["FocalLengthX"], camera.focal_x)) {
    return false;
  }

  if (!::read<double>(node["FocalLengthY"], camera.focal_y)) {
    return false;
  }

  if (!::read<double>(node["PrincipalPointX"], camera.principal_point.x)) {
    return false;
  }

  if (!::read<double>(node["PrincipalPointY"], camera.principal_point.y)) {
    return false;
  }

  if (!::read<double>(node["DistW"], camera.distort_w)) {
    return false;
  }

  return true;
}
