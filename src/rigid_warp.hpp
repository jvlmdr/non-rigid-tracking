#ifndef RIGID_WARP_HPP_
#define RIGID_WARP_HPP_

#include "warp.hpp"

class RigidWarpFunction {
  public:
    explicit RigidWarpFunction(double patch_size);

    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const;

  private:
    double resolution_;
};

class RigidWarp : public Warp {
  private:
    // Need resolution of patch to calculate scale.
    int resolution_;

    static const int NUM_PARAMS = 4;
    ceres::AutoDiffCostFunction<RigidWarpFunction, 2, 2, NUM_PARAMS> warp_;

  public:
    RigidWarp(int patch_size);
    ~RigidWarp();

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

#include "rigid_warp.inl"

#endif
