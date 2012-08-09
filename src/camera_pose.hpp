#ifndef CAMERA_POSE_HPP_
#define CAMERA_POSE_HPP_

#include <opencv2/core/core.hpp>

// Matrix representaiton of camera pose.
struct CameraPose {
  cv::Matx33d rotation; // using constant-size matrix puts memory on stack
  cv::Point3d center;
};

cv::Mat projectionMatrixFromCameraPose(const CameraPose& pose);

#endif
