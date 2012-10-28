#include "camera_properties.hpp"

cv::Matx33d CameraProperties::matrix() const {
  const double& fx = focal_x;
  const double& fy = focal_y;
  const double& px = principal_point.x;
  const double& py = principal_point.y;

  return cv::Matx33d(fx,  0, px,
                      0, fy, py,
                      0,  0,  1);
}
