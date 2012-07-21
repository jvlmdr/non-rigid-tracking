#ifndef TRANSLATION_WARP_HPP_
#define TRANSLATION_WARP_HPP_

#include "warp.hpp"

class TranslationWarpFunction {
  public:
    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const;
};

class TranslationWarp : public Warp {
  private:
    static const int NUM_PARAMS = 2;
    ceres::AutoDiffCostFunction<TranslationWarpFunction, 2, 2,
                                NUM_PARAMS> warp_;

  public:
    TranslationWarp();
    ~TranslationWarp();

    int numParams() const;

    cv::Point2d evaluate(const cv::Point2d& position,
                         const double* params,
                         double* jacobian) const;

    cv::Mat matrix(const double* params) const;

    void draw(cv::Mat& image,
              const double* params,
              int width,
              const cv::Scalar& color) const;
};

#include "translation_warp.inl"

#endif
