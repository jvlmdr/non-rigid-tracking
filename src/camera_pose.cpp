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

cv::Point3d CameraPose::directionOfRayThrough(const cv::Point2d& w) const {
  // Find vector in nullspace of linear projection system, A = R_xy - w R_z.
  cv::Mat R(rotation);
  cv::Mat Q = R.rowRange(0, 2) - cv::Mat(w) * R.rowRange(2, 3);

  // 1D nullspace found trivially by cross-product.
  // Take negative i x j because z < 0 is in front of camera.
  cv::Mat V = -Q.row(0).t().cross(Q.row(1).t());
  cv::Point3d v(V.at<double>(0, 0), V.at<double>(0, 1), V.at<double>(0, 2));

  return v;
}
