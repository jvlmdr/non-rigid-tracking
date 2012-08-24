#include "similarity_warp.hpp"
#include <cmath>

SimilarityWarpParams::SimilarityWarpParams() {}

SimilarityWarpParams::SimilarityWarpParams(double x,
                                           double y,
                                           double log_scale,
                                           double theta)
      : x(x), y(y), log_scale(log_scale), theta(theta) {}

////////////////////////////////////////////////////////////////////////////////

SimilarityWarp::SimilarityWarp() : warp_(new SimilarityWarpFunction()) {}

SimilarityWarp::~SimilarityWarp() {}

cv::Point2d SimilarityWarp::evaluate(const cv::Point2d& position,
                                     const double* params,
                                     double* jacobian) const {
  // Get derivative of warp function.
  double x[2] = { position.x, position.y };
  // Create somewhere to write out the result.
  double y[2];

  const double* param_blocks[2] = { x, params };
  double* jacobian_blocks[2] = { NULL, jacobian };

  // Use Ceres to get the derivative of the warp function.
  warp_.Evaluate(param_blocks, y, jacobian_blocks);

  return cv::Point2d(y[0], y[1]);
}

int SimilarityWarp::numParams() const {
  return NUM_PARAMS;
}

cv::Mat SimilarityWarp::matrix(const double* params) const {
  SimilarityWarpParams p(params[0], params[1], params[2], params[3]);

  double scale = std::exp(p.log_scale);
  cv::Mat M = (cv::Mat_<double>(2, 3) <<
      scale * std::cos(p.theta), scale * -std::sin(p.theta), p.x,
      scale * std::sin(p.theta), scale *  std::cos(p.theta), p.y);

  return M;
}
