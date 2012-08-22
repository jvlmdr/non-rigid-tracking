#include "similarity_warp.hpp"
#include <cmath>
#include "similarity_feature.hpp"

const int LINE_THICKNESS = 2;

////////////////////////////////////////////////////////////////////////////////

SimilarityWarpFunction::SimilarityWarpFunction(double resolution)
    : resolution_(resolution) {}

////////////////////////////////////////////////////////////////////////////////

SimilarityWarp::SimilarityWarp(int resolution)
    : resolution_(resolution), warp_(new SimilarityWarpFunction(resolution)) {}

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
  const SimilarityFeature* p;
  p = reinterpret_cast<const SimilarityFeature*>(params);

  double scale = p->size / double(resolution_);
  cv::Mat M = (cv::Mat_<double>(2, 3) <<
      scale * std::cos(p->theta), scale * -std::sin(p->theta), p->x,
      scale * std::sin(p->theta), scale *  std::cos(p->theta), p->y);

  return M;
}

void SimilarityWarp::draw(cv::Mat& image,
                          const double* params,
                          int width,
                          const cv::Scalar& color) const {
  const SimilarityFeature* p;
  p = reinterpret_cast<const SimilarityFeature*>(params);

  const double NUM_STDDEV = 2;
  double radius = NUM_STDDEV * p->size / 2.;
  cv::Point2d c(0, 0);
  cv::Point2d b(0, NUM_STDDEV * resolution_ / 2.);
  c = evaluate(c, params, NULL);
  b = evaluate(b, params, NULL);

  cv::circle(image, c, radius, color, LINE_THICKNESS);
  cv::line(image, c, b, color, LINE_THICKNESS);
}
