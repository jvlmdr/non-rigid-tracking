#include "camera_properties_writer.hpp"

CameraPropertiesWriter::~CameraPropertiesWriter() {
}

void CameraPropertiesWriter::write(cv::FileStorage& file,
                                   const CameraProperties& camera) {
  file << "ImageWidth" << camera.image_size.width;
  file << "ImageHeight" << camera.image_size.height;
  file << "FocalLengthX" << camera.focal_x;
  file << "FocalLengthY" << camera.focal_y;
  file << "PrincipalPointX" << camera.principal_point.x;
  file << "PrincipalPointY" << camera.principal_point.y;
  file << "DistW" << camera.distort_w;
}
