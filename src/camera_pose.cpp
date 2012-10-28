#include "camera_pose.hpp"

cv::Matx34d CameraPose::matrix() const {
  cv::Matx34d matrix;

  // Take cv::Mat wrappers around cv::Matx objects.
  cv::Mat P(matrix, false);
  cv::Mat R(rotation, false);
  // Copy 3D vector into a matrix.
  cv::Mat c = (cv::Mat_<double>(3, 1) << center.x, center.y, center.z);
  cv::Mat t = -R * c;

  cv::Mat dst;
  dst = P.colRange(cv::Range(0, 3));
  R.copyTo(dst);
  dst = P.col(3);
  t.copyTo(dst);

  return matrix;
}
