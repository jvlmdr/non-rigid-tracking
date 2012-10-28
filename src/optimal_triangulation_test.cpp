#include <iostream>
#include <ctime>
#include <boost/random.hpp>
#include <opencv2/core/core.hpp>
#include <glog/logging.h>
#include "optimal_triangulation.hpp"
#include "geometry.hpp"
#include "util.hpp"

double fundamentalMatrixResidual(const cv::Matx33d& F,
                                 const cv::Point2d& x1,
                                 const cv::Point2d& x2) {
  cv::Mat F_mat(F, false);
  cv::Mat x1_mat = (cv::Mat_<double>(3, 1) << x1.x, x1.y, 1);
  cv::Mat x2_mat = (cv::Mat_<double>(3, 1) << x2.x, x2.y, 1);

  return sqr(x2_mat.dot(F_mat * x1_mat));
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

  cv::Matx33d F = computeFundMatFromCameras(P1, P2);

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
