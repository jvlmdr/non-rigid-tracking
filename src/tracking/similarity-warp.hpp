#ifndef TRACKING_SIMILARITY_WARP_HPP_
#define TRACKING_SIMILARITY_WARP_HPP_

#include "tracking/warp.hpp"
#include "tracking/similarity-warper.hpp"

namespace tracking {

class SimilarityWarp : public Warp {
  public:
    SimilarityWarp(double x,
                   double y,
                   double log_scale,
                   double theta);
    SimilarityWarp();
    ~SimilarityWarp();

    int numParams() const;

    cv::Point2d evaluate(const cv::Point2d& position, double* jacobian) const;
    cv::Mat matrix() const;
    bool isValid(const cv::Size& image_size, int patch_radius) const;

    void draw(cv::Mat& image,
              int radius,
              const cv::Scalar& color,
              int thickness) const;

    double* params();
    const double* params() const;
    Warper* newWarper() const;

    inline double x() const { return params_[0]; }
    inline double y() const { return params_[1]; }
    inline double logScale() const { return params_[2]; }
    inline double theta() const { return params_[3]; }

  private:
    std::vector<double> params_;

    static const int NUM_PARAMS = 4;
};

} // namespace tracking

#endif
