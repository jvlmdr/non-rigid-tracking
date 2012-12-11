#include <glog/logging.h>

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
  return imagePointToHomogeneous(x, 1);
}

inline cv::Mat imagePointToHomogeneous(const cv::Point2d& x, double w) {
  return (cv::Mat_<double>(3, 1) << x.x, x.y, w);
}

inline cv::Point2d imagePointFromHomogeneous(const cv::Mat& X) {
  return 1. / X.at<double>(2) * cv::Point2d(X.at<double>(0), X.at<double>(1));
}

inline cv::Mat worldPointToHomogeneous(const cv::Point3d& x) {
  return worldPointToHomogeneous(x, 1);
}

inline cv::Mat worldPointToHomogeneous(const cv::Point3d& x, double w) {
  return (cv::Mat_<double>(4, 1) << x.x, x.y, x.z, w);
}

inline cv::Point3d worldPointFromHomogeneous(const cv::Mat& X) {
  cv::Point3d x(X.at<double>(0), X.at<double>(1), X.at<double>(2));
  double w = X.at<double>(3);
  return 1. / w * x;
}

inline void imagePointsToHomogeneous(const std::vector<cv::Point2d>& points,
                                     cv::Mat& matrix) {
  int n = points.size();
  matrix.create(3, n, cv::DataType<double>::type);

  std::vector<cv::Point2d>::const_iterator point = points.begin();

  for (int i = 0; i < n; i += 1) {
    matrix.at<double>(0, i) = point->x;
    matrix.at<double>(1, i) = point->y;
    matrix.at<double>(2, i) = 1;

    ++point;
  }
}

inline void imagePointsFromHomogeneous(const cv::Mat& matrix,
                                       std::vector<cv::Point2d>& points) {
  CHECK(matrix.rows == 3);
  CHECK(matrix.type() == cv::DataType<double>::type);

  int n = matrix.cols;
  points.clear();

  for (int i = 0; i < n; i += 1) {
    double x = matrix.at<double>(0, i);
    double y = matrix.at<double>(1, i);
    double z = matrix.at<double>(2, i);

    points.push_back(cv::Point2d(x / z, y / z));
  }
}

inline cv::Point2d affineTransformImagePoint(const cv::Point2d& x,
                                             const cv::Mat& A) {
  return imagePointFromHomogeneous(A * imagePointToHomogeneous(x));
}

template<class T>
T* takeAddress(T& x) {
  return &x;
}

template<class U, class T>
U cast(const T& t) {
  return static_cast<U>(t);
}

template<class U, class T>
U* takeAddressAndCast(T& x) {
  return static_cast<U*>(&x);
}
