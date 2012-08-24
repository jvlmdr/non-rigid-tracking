#ifndef FLOW_HPP_
#define FLOW_HPP_

#include <opencv2/core/core.hpp>
#include "warp.hpp"

struct FlowOptions {
  int interpolation;
  ceres::Solver::Options solver_options;
  bool iteration_limit_is_fatal;
  bool check_condition;
  double max_condition;
};

bool trackPatch(const Warp& warp,
                const cv::Mat& reference,
                const cv::Mat& image,
                const cv::Mat& ddx_image,
                const cv::Mat& ddy_image,
                double* params,
                const cv::Mat& mask,
                const FlowOptions& options,
                const WarpValidator* validator);

#endif
