#ifndef CAMERA_PROPERTIES_HPP_
#define CAMERA_PROPERTIES_HPP_

#include <opencv2/core/core.hpp>

struct CameraProperties {
  cv::Size image_size;
  double focal_x;
  double focal_y;
  cv::Point2d principal_point;
  double distort_w;
};

// Construct intrinsic matrix.
cv::Mat intrinsicMatrixFromCameraProperties(const CameraProperties& camera);

#endif
