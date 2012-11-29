#ifndef TRANSLATION_WARP_HPP_
#define TRANSLATION_WARP_HPP_

#include "warp.hpp"

// Templated function to use with ceres' auto-differentiation.
class TranslationWarpFunction {
  public:
    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const {
      // x' = x + tx
      // y' = y + ty
      q[0] = x[0] + p[0];
      q[1] = x[1] + p[1];

      return true;
    }
};

class TranslationWarper : public Warper {
  public:
    static const int NUM_PARAMS = 2;

    typedef
        ceres::AutoDiffCostFunction<TranslationWarpFunction, 2, 2, NUM_PARAMS>
        CostFunction;

    TranslationWarper(const CostFunction& function);
    TranslationWarper();
    ~TranslationWarper();

    int numParams() const;

    cv::Point2d evaluate(const cv::Point2d& position,
                         const double* params,
                         double* jacobian) const;

    cv::Mat matrix(const double* params) const;

    bool isValid(const double* params,
                 const cv::Size& image_size,
                 int radius) const;

  private:
    const CostFunction* function_;
};

#endif
