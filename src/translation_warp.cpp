#include "translation_warp.hpp"

TranslationWarpParams::TranslationWarpParams() : x(0), y(0) {}

TranslationWarpParams::TranslationWarpParams(double x, double y) : x(x), y(y) {}

////////////////////////////////////////////////////////////////////////////////

TranslationWarp::TranslationWarp() : warp_(new TranslationWarpFunction) {}

TranslationWarp::~TranslationWarp() {}

int TranslationWarp::numParams() const {
  return 2;
}

cv::Point2d TranslationWarp::evaluate(const cv::Point2d& position,
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

cv::Mat TranslationWarp::matrix(const double* params) const {
  TranslationWarpParams p(params[0], params[1]);
  cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, p->x, 0, 1, p->y);
  return M;
}
