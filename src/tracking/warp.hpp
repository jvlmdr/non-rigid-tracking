#ifndef TRACKING_WARP_HPP_
#define TRACKING_WARP_HPP_

#include "tracking/using.hpp"
#include "tracking/warper.hpp"

namespace tracking {

// Describes a continuously-parametrized affine warp.
//
// Has:
// - number of parameters
// - method to transform a point
// - method to produce an affine matrix
// - parameters
class Warp {
  public:
    virtual ~Warp() {}

    // Returns the number of parameters.
    virtual int numParams() const = 0;

    // Evaluates the warp, finds the Jacobian with respect to warp parameters.
    virtual cv::Point2d evaluate(const cv::Point2d& position,
                                 double* jacobian) const = 0;

    // Returns a matrix representation of the affine warp.
    // For use with warpAffine().
    virtual cv::Mat matrix() const = 0;

    // Returns whether the warp is valid.
    virtual bool isValid(const cv::Size& image_size, int radius) const = 0;

    // Draws the warp on an image.
    virtual void draw(cv::Mat& image,
                      int radius,
                      const cv::Scalar& color,
                      int thickness) const = 0;

    // Provides access to the parameters.
    virtual double* params() = 0;
    virtual const double* params() const = 0;

    // Allocate and construct a warper.
    virtual Warper* newWarper() const = 0;
};

// Extracts a square patch of an image after applying a warp.
void samplePatch(const Warp& warp,
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

} // namespace tracking

#endif
