#include "camera_writer.hpp"
#include "camera_properties_writer.hpp"
#include "camera_pose_writer.hpp"

CameraWriter::~CameraWriter() {}

void CameraWriter::write(cv::FileStorage& file, const Camera& camera) {
  CameraPropertiesWriter properties_writer;
  properties_writer.write(file, camera.intrinsics());

  CameraPoseWriter pose_writer;
  pose_writer.write(file, camera.extrinsics());
}
