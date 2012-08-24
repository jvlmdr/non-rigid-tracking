#ifndef WARP_HPP_
#define WARP_HPP_

#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

// Describes a parametrized affine warp.
//
// Has:
// - number of parameters
// - radius
// - method to transform a point given parameters
// - method to produce an affine matrix from the parameters
// - method to draw a warped patch on an image
class Warp {
  public:
    virtual ~Warp() {}

    // Returns the number of parameters.
    virtual int numParams() const = 0;

    // Evaluates the warp, finds the Jacobian with respect to warp parameters.
    virtual cv::Point2d evaluate(const cv::Point2d& position,
                                 const double* params,
                                 double* jacobian) const = 0;

    // Returns a matrix representation of the affine warp.
    // For use with warpAffine().
    virtual cv::Mat matrix(const double* params) const = 0;
};

class WarpValidator {
  public:
    virtual ~WarpValidator() {}

    // Checks if a set of parameters are valid.
    virtual bool check(const double* params) const = 0;
};

// Extracts a square patch of an image after applying a warp.
void samplePatch(const Warp& warp,
                 const double* params,
                 const cv::Mat& image,
                 cv::Mat& patch,
                 int width,
                 bool invert,
                 int interpolation);

// Extracts a square patch of an image after applying a warp.
void samplePatchAffine(const cv::Mat& src,
                       cv::Mat& dst,
                       const cv::Mat& M,
                       int width,
                       bool invert,
                       int interpolation);

#endif
