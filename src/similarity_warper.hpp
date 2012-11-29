#ifndef SIMILARITY_WARPER_HPP_
#define SIMILARITY_WARPER_HPP_

#include "warper.hpp"

// Templated function to use with ceres' auto-differentiation.
class SimilarityWarpFunction {
  public:
    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const {
      // x' = scale * (cos(theta) * x + -sin(theta) * y) + tx
      // y' = scale * (sin(theta) * x +  cos(theta) * y) + ty
      q[0] = exp(p[2]) * (cos(p[3]) * x[0] - sin(p[3]) * x[1]) + p[0];
      q[1] = exp(p[2]) * (sin(p[3]) * x[0] + cos(p[3]) * x[1]) + p[1];

      return true;
    }
};

class SimilarityWarper : public Warper {
  public:
    static const int NUM_PARAMS = 4;

    typedef
        ceres::AutoDiffCostFunction<SimilarityWarpFunction, 2, 2, NUM_PARAMS>
        CostFunction;

    SimilarityWarper(const CostFunction& function, double min_scale);
    SimilarityWarper();
    ~SimilarityWarper();

    int numParams() const;

    cv::Point2d evaluate(const cv::Point2d& position,
                         const double* params,
                         double* jacobian) const;

    cv::Mat matrix(const double* params) const;

    bool isValid(const double* params,
                 const cv::Size& image_size,
                 int patch_radius) const;

  private:
    // Cost function.
    const CostFunction* function_;
    // For checking validity.
    double min_scale_;
};

#endif
