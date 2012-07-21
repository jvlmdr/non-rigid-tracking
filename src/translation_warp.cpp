#include "translation_warp.hpp"
#include "translation_feature.hpp"

const int LINE_THICKNESS = 2;

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
  const TranslationFeature* p =
      reinterpret_cast<const TranslationFeature*>(params);

  cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, p->x, 0, 1, p->y);

  return M;
}

void TranslationWarp::draw(cv::Mat& image,
                           const double* params,
                           int width,
                           const cv::Scalar& color) const {
  const TranslationFeature* p =
      reinterpret_cast<const TranslationFeature*>(params);

  cv::Point2d c(p->x, p->y);
  cv::Point2d i(1, 0);
  cv::Point2d j(0, 1);
  double radius = (width - 1) / 2.;
  i *= radius;
  j *= radius;

  cv::line(image, c - i - j, c + i - j, color, LINE_THICKNESS);
  cv::line(image, c - i + j, c + i + j, color, LINE_THICKNESS);
  cv::line(image, c - i - j, c - i + j, color, LINE_THICKNESS);
  cv::line(image, c + i - j, c + i + j, color, LINE_THICKNESS);
  cv::circle(image, c, LINE_THICKNESS, color, -1);
}
