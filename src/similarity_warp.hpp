#ifndef SIMILARITY_WARP_HPP_
#define SIMILARITY_WARP_HPP_

#include "warp.hpp"

class SimilarityWarpFunction {
  public:
    explicit SimilarityWarpFunction(double patch_size);

    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const;

  private:
    double resolution_;
};

class SimilarityWarp : public Warp {
  private:
    // Need resolution of patch to calculate scale.
    int resolution_;

    static const int NUM_PARAMS = 4;
    ceres::AutoDiffCostFunction<SimilarityWarpFunction, 2, 2, NUM_PARAMS> warp_;

  public:
    SimilarityWarp(int patch_size);
    ~SimilarityWarp();

    cv::Point2d evaluate(const cv::Point2d& position,
                         const double* params,
                         double* jacobian) const;

    int numParams() const;

    cv::Mat matrix(const double* params) const;

    void draw(cv::Mat& image,
              const double* params,
              int width,
              const cv::Scalar& color) const;
};

#include "similarity_warp.inl"

#endif
