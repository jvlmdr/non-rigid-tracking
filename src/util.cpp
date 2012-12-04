#include "util.hpp"
#include <fstream>

double cond(const cv::Mat& A) {
  // Compute condition number.
  std::vector<double> sigma;
  cv::SVD::compute(A, sigma, cv::SVD::NO_UV);
  return sigma.front() / sigma.back();
}

bool fileExists(const std::string& filename) {
  return std::ifstream(filename.c_str()).good();
}
