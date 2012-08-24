inline double sqr(double x) {
  return x * x;
}

inline bool isNumber(double x) {
  return (x == x);
}

inline bool isFinite(double x) {
  return (x <= std::numeric_limits<double>::max() &&
          x >= -std::numeric_limits<double>::max());
}

inline cv::Mat imagePointToHomogeneous(const cv::Point2d& x) {
  return (cv::Mat_<double>(3, 1) << x.x, x.y, 1);
}

inline cv::Point2d imagePointFromHomogeneous(const cv::Mat& X) {
  return 1. / X.at<double>(2) * cv::Point2d(X.at<double>(0), X.at<double>(1));
}

inline cv::Mat worldPointToHomogeneous(const cv::Point3d& x) {
  return (cv::Mat_<double>(4, 1) << x.x, x.y, x.z, 1);
}

inline cv::Point3d worldPointFromHomogeneous(const cv::Mat& X) {
  cv::Point3d x(X.at<double>(0), X.at<double>(1), X.at<double>(2));
  double w = X.at<double>(3);
  return 1. / w * x;
}
