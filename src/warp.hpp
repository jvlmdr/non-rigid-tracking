#ifndef WARP_HPP_
#define WARP_HPP_

#include <opencv2/core/core.hpp>
// TODO: Separate differentiation from rest of implementation?
// Otherwise anything that uses a warp has to...
#include <ceres/ceres.h>

// Describes a parametrized affine warp.
//
// Has:
// - number of parameters
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

    // Visualizes a patch on an image.
    virtual void draw(cv::Mat& image,
                      const double* params,
                      int width,
                      const cv::Scalar& color) const = 0;
};

#endif
