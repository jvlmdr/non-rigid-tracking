#include "camera_properties_reader.hpp"

CameraPropertiesReader::~CameraPropertiesReader() {}

void CameraPropertiesReader::read(const cv::FileNode& node,
                                  CameraProperties& camera) {
  camera.image_size.width = static_cast<int>(node["ImageWidth"]);
  camera.image_size.height = static_cast<int>(node["ImageHeight"]);
  camera.focal_x = static_cast<double>(node["FocalLengthX"]);
  camera.focal_y = static_cast<double>(node["FocalLengthY"]);
  camera.principal_point.x = static_cast<double>(node["PrincipalPointX"]);
  camera.principal_point.y = static_cast<double>(node["PrincipalPointY"]);
  camera.distort_w = static_cast<double>(node["DistW"]);
}
