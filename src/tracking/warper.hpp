#ifndef TRACKING_WARPER_HPP_
#define TRACKING_WARPER_HPP_

#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

namespace tracking {

// Describes a class of parametrized affine warps.
//
// Has:
// - number of parameters
// - method to transform a point given parameters
// - method to produce an affine matrix given parameters
//
// Warper exists distinct from Warp because ceres needs a function which
// operates on double*, while users would rather work with x, y, etc.
//
// The functionality in Warper is no more than required by WarpCost::Evaluate.
class Warper {
  public:
    virtual ~Warper() {}

    // Returns the number of parameters.
    virtual int numParams() const = 0;

    // Evaluates the warp, finds the Jacobian with respect to warp parameters.
    virtual cv::Point2d evaluate(const cv::Point2d& position,
                                 const double* params,
                                 double* jacobian) const = 0;

    // Returns a matrix representation of the affine warp.
    // For use with warpAffine().
    virtual cv::Mat matrix(const double* params) const = 0;

    // Returns whether the warp is valid.
    virtual bool isValid(const double* params,
                         const cv::Size& image_size,
                         int radius) const = 0;
};

}

#endif
