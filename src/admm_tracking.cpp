#include "admm_tracking.hpp"
#include "util.hpp"

#include <opencv2/highgui/highgui.hpp>

// z is 1x2, u is 1x2, x is 1x2
void findBestResponse(const cv::Mat& z,
                      const cv::Mat& u,
                      const cv::Mat& response,
                      double rho,
                      cv::Mat& x) {
  cv::Size size = response.size();

  cv::Mat v = z - u;
  double vx = v.at<double>(0, 0);
  double vy = v.at<double>(0, 1);

  // Compute distance from v.
  cv::Mat distance = cv::Mat_<double>(size);
  for (int i = 0; i < size.height; i += 1) {
    for (int j = 0; j < size.width; j += 1) {
      distance.at<double>(i, j) = sqr(i - vy) + sqr(j - vx);
    }
  }

  // Add distance to response.
  cv::Mat cost = response + rho / 2. * distance;

  // Find minimum.
  cv::Point arg;
  double min;
  cv::minMaxLoc(cost, &min, NULL, &arg, NULL);

  x.at<double>(0, 0) = arg.x;
  x.at<double>(0, 1) = arg.y;
}

// x is nx1, u is nx1, z is nx1
void findSmoothestPath(const cv::Mat& x,
                       const cv::Mat& u,
                       double rho,
                       double lambda,
                       cv::Mat& z) {
  int n = x.total();

  // Set up big linear system.
  cv::Mat D = cv::Mat_<double>(n - 1, n, 0.);
  for (int t = 0; t < n; t += 1) {
    D.at<double>(t, t) = 1;
    D.at<double>(t, t + 1) = -1;
  }

  cv::Mat I = cv::Mat_<double>::eye(n, n);
  cv::Mat A = I + 2. * lambda / rho * D.t() * D;
  cv::Mat v = x + u;

  z = A.inv() * v;
}

double findClassifierTrackAdmm(const std::vector<cv::Mat>& responses,
                               std::vector<cv::Point2d>& positions,
                               double lambda,
                               double rho) {
  int n = responses.size();

  // Guess Lagrange multipliers to all be zero.
  cv::Mat x = cv::Mat_<double>(n, 2, 0.);
  // Initialize track positions to origin.
  cv::Mat z = cv::Mat_<double>(n, 2, 0.);
  // Smoothest path will also be origin.
  cv::Mat u = cv::Mat_<double>(n, 2, 0.);

  while (true) {
    // Solve first sub-problem.
    for (int t = 0; t < n; t += 1) {
      cv::Mat dst = x.row(t);
      findBestResponse(z.row(t), u.row(t), responses[t], rho, dst);
    }
    LOG(INFO) << "x-update:";
    LOG(INFO) << "norm(x - z) => " << cv::norm(x - z);

    // Solve second sub-problem.
    for (int d = 0; d < 2; d += 1) {
      cv::Mat dst = z.col(d);
      findSmoothestPath(x.col(d), u.col(d), rho, lambda, dst);
    }
    LOG(INFO) << "z-update:";
    LOG(INFO) << "norm(x - z) => " << cv::norm(x - z);

    // Update multipliers.
    u += x - z;
  }
}
