#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "camera_properties.hpp"
#include "camera_pose.hpp"
#include <opencv2/core/core.hpp>

class Camera {
  public:
    Camera();
    Camera(const Camera& other);
    Camera(const CameraProperties& intrinsics, const CameraPose& extrinsics);

    const CameraProperties& intrinsics() const;
    const CameraPose& extrinsics() const;
    cv::Matx34d matrix() const;

    cv::Point2d project(const cv::Point3d& x) const;

  private:
    CameraProperties intrinsics_;
    CameraPose extrinsics_;
    cv::Matx34d matrix_;
};

#endif
