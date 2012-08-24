#ifndef SIMILARITY_WARP_HPP_
#define SIMILARITY_WARP_HPP_

#include "warp.hpp"

struct SimilarityWarpParams {
  double x;
  double y;
  double log_scale;
  double theta;

  SimilarityWarpParams();
  SimilarityWarpParams(double x, double y, double log_scale, double theta);
};

class SimilarityWarpFunction {
  public:
    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const;
};

class SimilarityWarp : public Warp {
  private:
    static const int NUM_PARAMS = 4;
    ceres::AutoDiffCostFunction<SimilarityWarpFunction, 2, 2, NUM_PARAMS> warp_;

  public:
    SimilarityWarp();
    ~SimilarityWarp();

    int numParams() const;
    cv::Point2d evaluate(const cv::Point2d& position,
                         const double* params,
                         double* jacobian) const;
    cv::Mat matrix(const double* params) const;
};

#include "similarity_warp.inl"

#endif
