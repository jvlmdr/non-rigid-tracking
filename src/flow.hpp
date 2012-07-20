#ifndef FLOW_HPP_
#define FLOW_HPP_

#include <opencv2/core/core.hpp>
#include "warp.hpp"

bool solveFlow(const Warp& warp,
               int patch_size,
               const cv::Mat& reference,
               const cv::Mat& image,
               const cv::Mat& ddx_image,
               const cv::Mat& ddy_image,
               double* params,
               const ceres::Solver::Options& options,
               bool iteration_limit_is_fatal,
               double max_condition);

void sampleAffinePatch(const cv::Mat& src,
                       cv::Mat& dst,
                       const cv::Mat& M,
                       int patch_size);

double meanPixelDifference(const Warp& warp,
                           int patch_size,
                           const cv::Mat& reference,
                           const cv::Mat& image,
                           const double* params);

#endif
