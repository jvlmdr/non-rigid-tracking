#ifndef CAMERA_PROPERTIES_HPP_
#define CAMERA_PROPERTIES_HPP_

#include <opencv2/core/core.hpp>
#include "axis_aligned_ellipse.hpp"

struct CameraProperties {
  cv::Size image_size;
  double focal_x;
  double focal_y;
  cv::Point2d principal_point;
  double distort_w;

  cv::Matx33d matrix() const;

  cv::Point2d calibrate(const cv::Point2d& y) const;
  cv::Point2d uncalibrate(const cv::Point2d& x) const;
  cv::Point2d calibrateAndUndistort(const cv::Point2d& y) const;
  cv::Point2d distortAndUncalibrate(const cv::Point2d& x) const;

  // Returns the undistortable region of the uncalibrated, distorted image.
  AxisAlignedEllipse undistortableRegion() const;

  cv::Rect bounds() const;
};

#endif
