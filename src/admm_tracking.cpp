#include "admm_tracking.hpp"
#include "util.hpp"

#include <opencv2/highgui/highgui.hpp>

void findBestResponse(const cv::Point2d& z,
                      const cv::Point2d& u,
                      const cv::Mat& response,
                      double rho,
                      cv::Point2d& x) {
  cv::Size size = response.size();

  cv::Point2d v = z - u;

  // Compute distance from v.
  cv::Mat distance = cv::Mat_<double>(size);
  for (int i = 0; i < size.height; i += 1) {
    for (int j = 0; j < size.width; j += 1) {
      distance.at<double>(i, j) = sqr(v.x - j) + sqr(v.y - i);
    }
  }

  // Add distance to response.
  cv::Mat cost = response + rho / 2. * distance;

  // Find minimum.
  double min;
  cv::Point loc;
  cv::minMaxLoc(cost, &min, NULL, &loc, NULL);
  cv::Point2d p = loc;

  // Vector from p to v.
  cv::Point2d r = v - p;

  // Restrict to half-pixel trust region.
  double d = cv::norm(r);
  if (d > 0.5) {
    x = p + 0.5 / d * r;
  } else {
    x = v;
  }
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
      cv::Point2d z_t(z.at<double>(t, 0), z.at<double>(t, 1));
      cv::Point2d u_t(u.at<double>(t, 0), u.at<double>(t, 1));

      cv::Point2d x_t;
      findBestResponse(z_t, u_t, responses[t], rho, x_t);

      x.at<double>(t, 0) = x_t.x;
      x.at<double>(t, 1) = x_t.y;
    }
    LOG(INFO) << "x-update:";
    LOG(INFO) << "norm(x - z) => " << cv::norm(x - z) / n;

    // Solve second sub-problem.
    for (int d = 0; d < 2; d += 1) {
      cv::Mat dst = z.col(d);
      findSmoothestPath(x.col(d), u.col(d), rho, lambda, dst);
    }
    LOG(INFO) << "z-update:";
    LOG(INFO) << "norm(x - z) => " << cv::norm(x - z);

    // Update multipliers.
    u += x - z;

    LOG(INFO) << "x[0] => " << x.row(0);
    LOG(INFO) << "z[0] => " << z.row(0);
    LOG(INFO);
    LOG(INFO) << "x[1] => " << x.row(1);
    LOG(INFO) << "z[1] => " << z.row(1);
    LOG(INFO);
    LOG(INFO) << "x[2] => " << x.row(2);
    LOG(INFO) << "z[2] => " << z.row(2);
    LOG(INFO);
  }
}
