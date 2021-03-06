#include "camera.hpp"

Camera::Camera() : intrinsics_(), extrinsics_() {}

Camera::Camera(const Camera& other) : intrinsics_(other.intrinsics_),
                                      extrinsics_(other.extrinsics_) {}

Camera::Camera(const CameraProperties& intrinsics, const CameraPose& extrinsics)
    : intrinsics_(intrinsics), extrinsics_(extrinsics) {}

const CameraProperties& Camera::intrinsics() const {
  return intrinsics_;
}

const CameraPose& Camera::extrinsics() const {
  return extrinsics_;
}

cv::Matx34d Camera::matrix() const {
  return intrinsics_.matrix() * extrinsics_.matrix();
}

cv::Point2d Camera::project(const cv::Point3d& x) const {
  cv::Point2d w = extrinsics_.project(x);
  return intrinsics_.distortAndUncalibrate(w);
}
