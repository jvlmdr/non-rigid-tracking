#include "camera_properties.hpp"

cv::Mat intrinsicMatrixFromCameraProperties(const CameraProperties& camera) {
  const double& fx = camera.focal_x;
  const double& fy = camera.focal_y;
  const double& px = camera.principal_point.x;
  const double& py = camera.principal_point.y;

  return (cv::Mat_<double>(3, 3) <<
      fx,  0, px,
       0, fy, py,
       0,  0,  1);
}
