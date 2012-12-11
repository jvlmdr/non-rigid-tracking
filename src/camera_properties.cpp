#include "camera_properties.hpp"
#include <cmath>
#include "distortion.hpp"
#include "util.hpp"

cv::Matx33d CameraProperties::matrix() const {
  const double& fx = focal_x;
  const double& fy = focal_y;
  const double& px = principal_point.x;
  const double& py = principal_point.y;

  return cv::Matx33d(fx,  0, px,
                      0, fy, py,
                      0,  0,  1);
}

cv::Point2d CameraProperties::calibrate(const cv::Point2d& y) const {
  // Multiply by K inverse.
  cv::Mat Y = imagePointToHomogeneous(y);
  cv::Mat X = cv::Mat(matrix().inv()) * Y;
  return imagePointFromHomogeneous(X);
}

cv::Point2d CameraProperties::uncalibrate(const cv::Point2d& x) const {
  // Multiply by K.
  cv::Mat X = imagePointToHomogeneous(x);
  cv::Mat Y = cv::Mat(matrix()) * X;
  return imagePointFromHomogeneous(Y);
}

cv::Point2d CameraProperties::calibrateAndUndistort(
    const cv::Point2d& y) const {
  // Multiply by K inverse.
  cv::Mat Y = imagePointToHomogeneous(y);
  cv::Point2d x = imagePointFromHomogeneous(cv::Mat(matrix().inv()) * Y);
  // Undistort.
  return undistort(x, distort_w);
}

cv::Point2d CameraProperties::distortAndUncalibrate(
    const cv::Point2d& x) const {
  // Distort.
  cv::Point2d y = distort(x, distort_w);
  // Multiply by K.
  cv::Mat Y = imagePointToHomogeneous(y);
  return imagePointFromHomogeneous(cv::Mat(matrix()) * Y);
}

AxisAlignedEllipse CameraProperties::undistortableRegion() const {
  // Get the circular bounds in the calibrated, distorted image.
  double r = maxDistortedRadius(distort_w);

  // Apply intrinsics to get ellipse in uncalibrated image.
  double a = r * std::abs(focal_x);
  double b = r * std::abs(focal_y);

  return AxisAlignedEllipse(a, b, principal_point);
}

cv::Rect CameraProperties::bounds() const {
  return cv::Rect(cv::Point(0, 0), image_size);
}
