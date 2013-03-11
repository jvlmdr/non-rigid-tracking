#ifndef TRACKING_TRANSLATION_WARP_HPP_
#define TRACKING_TRANSLATION_WARP_HPP_

#include "tracking/warp.hpp"
#include "tracking/translation-warper.hpp"

namespace tracking {

class TranslationWarp : public Warp {
  public:
    TranslationWarp(double x, double y);
    TranslationWarp();
    ~TranslationWarp();

    int numParams() const;

    cv::Point2d evaluate(const cv::Point2d& position, double* jacobian) const;
    cv::Mat matrix() const;
    bool isValid(const cv::Size& image_size, int radius) const;

    void draw(cv::Mat& image,
              int radius,
              const cv::Scalar& color,
              int thickness) const;

    double* params();
    const double* params() const;
    Warper* newWarper() const;

    inline double x() const { return params_[0]; }
    inline double y() const { return params_[1]; }

  private:
    std::vector<double> params_;

    static const int NUM_PARAMS = 2;
};

}

#endif
