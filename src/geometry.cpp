#include "geometry.hpp"

cv::Point2d project(const cv::Mat& P, const cv::Point3d& x) {
  cv::Mat A = P(cv::Range::all(), cv::Range(0, 3));
  cv::Mat B = P.col(3);
  cv::Mat X(x, false);

  cv::Mat W = A * X + B;
  double u = W.at<double>(0);
  double v = W.at<double>(1);
  double z = W.at<double>(2);

  return cv::Point2d(u / z, v / z);
}

cv::Mat computeFundMatFromCameras(const cv::Mat& P1, const cv::Mat& P2) {
  cv::Mat F = cv::Mat_<double>(3, 3);

  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < 3; j += 1) {
      int r = 0;

      // Build matrix of rows of P1 and P2 that aren't i and j respectively.
      cv::Mat A = cv::Mat_<double>(4, 4);
      for (int k = 0; k < 3; k += 1) {
        if (k != i) {
          cv::Mat dst = A.row(r);
          P1.row(k).copyTo(dst);
          r += 1;
        }
      }
      for (int k = 0; k < 3; k += 1) {
        if (k != j) {
          cv::Mat dst = A.row(r);
          P2.row(k).copyTo(dst);
          r += 1;
        }
      }

      // Find determinant of this matrix.
      double f = cv::determinant(A);

      // Flip sign if (i + j) is odd.
      if ((i + j) % 2 != 0) {
        f = -f;
      }

      F.at<double>(j, i) = f;
    }
  }

  return F;
}
