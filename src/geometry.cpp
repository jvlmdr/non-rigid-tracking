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

cv::Matx33d computeFundMatFromCameras(const cv::Matx34d& P1,
                                      const cv::Matx34d& P2) {
  cv::Matx33d F;
  cv::Mat P1_mat(P1, false);
  cv::Mat P2_mat(P2, false);

  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < 3; j += 1) {
      int r = 0;

      // Build matrix of rows of P1 and P2 that aren't i and j respectively.
      cv::Mat A = cv::Mat_<double>(4, 4);
      for (int k = 0; k < 3; k += 1) {
        if (k != i) {
          cv::Mat dst = A.row(r);
          P1_mat.row(k).copyTo(dst);
          r += 1;
        }
      }
      for (int k = 0; k < 3; k += 1) {
        if (k != j) {
          cv::Mat dst = A.row(r);
          P2_mat.row(k).copyTo(dst);
          r += 1;
        }
      }

      // Find determinant of this matrix.
      double f = cv::determinant(A);

      // Flip sign if (i + j) is odd.
      if ((i + j) % 2 != 0) {
        f = -f;
      }

      F(j, i) = f;
    }
  }

  return F;
}
