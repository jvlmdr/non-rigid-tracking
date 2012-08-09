#include "camera_pose.hpp"

cv::Mat projectionMatrixFromCameraPose(const CameraPose& pose) {
  cv::Mat R(pose.rotation, false);
  cv::Mat c = (cv::Mat_<double>(3, 1) <<
      pose.center.x, pose.center.y, pose.center.z);
  cv::Mat t = -R * c;

  cv::Mat P = cv::Mat_<double>(3, 4);
  cv::Mat dst;
  dst = P.colRange(cv::Range(0, 3));
  R.copyTo(dst);
  dst = P.col(3);
  t.copyTo(dst);

  return P;
}
