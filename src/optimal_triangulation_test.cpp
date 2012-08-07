#include <iostream>
#include <ctime>
#include <boost/random.hpp>
#include <opencv2/core/core.hpp>
#include <glog/logging.h>
#include "optimal_triangulation.hpp"
#include "util.hpp"

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

double fundamentalMatrixResidual(const cv::Mat& F,
                                 const cv::Point2d& x1,
                                 const cv::Point2d& x2) {
  cv::Mat X1 = (cv::Mat_<double>(3, 1) << x1.x, x1.y, 1);
  cv::Mat X2 = (cv::Mat_<double>(3, 1) << x2.x, x2.y, 1);

  return sqr(X2.dot(F * X1));
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // Come up with some points to test the triangulation algorithm.
  boost::random::mt19937 rng;
  boost::random::normal_distribution<> dist(0, 1);

  rng.seed(static_cast<unsigned int>(std::clock()));

  cv::Point3d x(dist(rng), dist(rng), dist(rng));
  cv::Mat P1 = (cv::Mat_<double>(3, 4) <<
      dist(rng), dist(rng), dist(rng), dist(rng),
      dist(rng), dist(rng), dist(rng), dist(rng),
      dist(rng), dist(rng), dist(rng), dist(rng));
  cv::Mat P2 = (cv::Mat_<double>(3, 4) <<
      dist(rng), dist(rng), dist(rng), dist(rng),
      dist(rng), dist(rng), dist(rng), dist(rng),
      dist(rng), dist(rng), dist(rng), dist(rng));

  cv::Mat F = computeFundMatFromCameras(P1, P2);

  cv::Point2d w1 = project(P1, x) + 1e-1 * cv::Point2d(dist(rng), dist(rng));
  cv::Point2d w2 = project(P2, x) + 1e-1 * cv::Point2d(dist(rng), dist(rng));

  std::cout << "x: " << x << std::endl;
  std::cout << "w1: " << w1 << std::endl;
  std::cout << "w2: " << w2 << std::endl;
  std::cout << "fundamental matrix residual: " <<
    fundamentalMatrixResidual(F, w1, w2) << std::endl;
  std::cout << std::endl;

  cv::Point2d w1_hat = w1;
  cv::Point2d w2_hat = w2;
  double residual = optimalTriangulation(w1_hat, w2_hat, F);

  std::cout << "w1: " << w1_hat << std::endl;
  std::cout << "w2: " << w2_hat << std::endl;
  std::cout << "fundamental matrix residual: " <<
    fundamentalMatrixResidual(F, w1_hat, w2_hat) << std::endl;
  std::cout << std::endl;

  double distance = sqr(cv::norm(w1_hat - w1)) + sqr(cv::norm(w2_hat - w2));
  std::cout << "s(t*): " << residual << std::endl;
  std::cout << "d(x1, x1*) + d(x2, x2*): " << distance << std::endl;
  std::cout << std::endl;

  return 0;
}
