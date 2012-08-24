#include "util.hpp"

double cond(const cv::Mat& A) {
  // Compute condition number.
  std::vector<double> sigma;
  cv::SVD::compute(A, sigma, cv::SVD::NO_UV);
  return sigma.front() / sigma.back();
}
