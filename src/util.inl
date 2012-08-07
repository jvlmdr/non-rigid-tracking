inline double sqr(double x) {
  return x * x;
}

inline cv::Point2d convertPointFromHomogeneous2(const cv::Mat& X) {
  return 1. / X.at<double>(2) * cv::Point2d(X.at<double>(0), X.at<double>(1));
}
