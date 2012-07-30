#ifndef FLOW_HPP_
#define FLOW_HPP_

#include <opencv2/core/core.hpp>
#include "warp.hpp"

bool trackPatch(const Warp& warp,
                int patch_size,
                const cv::Mat& reference,
                const cv::Mat& image,
                const cv::Mat& ddx_image,
                const cv::Mat& ddy_image,
                double* params,
                const ceres::Solver::Options& options,
                bool iteration_limit_is_fatal,
                double max_condition);

class WarpTracker {
  public:
    // Sets the warp function and the optimization parameters.
    WarpTracker(const Warp& warp,
                int patch_size,
                int max_num_iterations,
                double function_tolerance,
                double gradient_tolerance,
                double parameter_tolerance,
                bool iteration_limit_is_fatal,
                double max_condition);

    // Feeds the next image into the tracker.
    void feedImage(const cv::Mat& image);

    // Tracks a feature from the previous image to the next.
    bool track(double* feature) const;

  private:
    const Warp* warp_;

    // Configuration.
    int patch_size_;
    ceres::Solver::Options options_;
    bool iteration_limit_is_fatal_;
    int max_condition_;

    // Previous image and gradients of current image.
    cv::Mat previous_image_;
    cv::Mat image_;
    cv::Mat ddx_image_;
    cv::Mat ddy_image_;
};

////////////////////////////////////////////////////////////////////////////////

void sampleAffine(const cv::Mat& src,
                  cv::Mat& dst,
                  const cv::Mat& M,
                  const cv::Size& size,
                  bool invert);

void sampleAffinePatch(const cv::Mat& src,
                       cv::Mat& dst,
                       const cv::Mat& M,
                       int patch_width,
                       bool invert);

double averageResidual(const cv::Mat& A, const cv::Mat& B);

#endif
