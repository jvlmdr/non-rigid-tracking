#ifndef CAMERA_POSE_HPP_
#define CAMERA_POSE_HPP_

#include <opencv2/core/core.hpp>

// Matrix representaiton of camera pose.
struct CameraPose {
  cv::Matx33d rotation; // using constant-size matrix puts memory on stack
  cv::Point3d center;

  cv::Matx34d matrix() const;

  cv::Point2d project(const cv::Point3d& x) const;
  cv::Point3d directionOfRayThrough(const cv::Point2d& w) const;
};

#endif
