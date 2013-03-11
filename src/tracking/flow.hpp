#ifndef TRACKING_FLOW_HPP_
#define TRACKING_FLOW_HPP_

#include "tracking/using.hpp"
#include "tracking/warp.hpp"

namespace tracking {

struct FlowOptions {
  int interpolation;
  ceres::Solver::Options solver_options;
  bool iteration_limit_is_fatal;
  bool check_condition;
  double max_condition;
};

bool trackPatch(Warp& warp,
                const cv::Mat& reference,
                const cv::Mat& image,
                const cv::Mat& ddx_image,
                const cv::Mat& ddy_image,
                const cv::Mat& mask,
                const FlowOptions& options);

}

#endif
