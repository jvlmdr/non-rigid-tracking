#ifndef SIMILARITY_WARP_HPP_
#define SIMILARITY_WARP_HPP_

#include "warp.hpp"
#include "similarity_warper.hpp"

class SimilarityWarp : public Warp {
  public:
    SimilarityWarp(double x,
                   double y,
                   double log_scale,
                   double theta,
                   const SimilarityWarper& warper);
    SimilarityWarp();
    ~SimilarityWarp();

    int numParams() const;

    cv::Point2d evaluate(const cv::Point2d& position, double* jacobian) const;
    cv::Mat matrix() const;
    bool isValid(const cv::Size& image_size, int patch_radius) const;

    double* params();
    const double* params() const;
    const Warper* warper() const;

    inline double x() const { return params_[0]; }
    inline double y() const { return params_[1]; }
    inline double logScale() const { return params_[2]; }
    inline double theta() const { return params_[3]; }

  private:
    // Warp parameters.
    std::vector<double> params_;
    // Underlying warper.
    const SimilarityWarper* warper_;

    static const int NUM_PARAMS = 4;
};

#endif
