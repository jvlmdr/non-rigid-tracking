#include "warp.hpp"
#include <opencv2/imgproc/imgproc.hpp>

void samplePatch(const Warp& warp,
                 const cv::Mat& image,
                 cv::Mat& patch,
                 int width,
                 bool invert,
                 int interpolation) {
  cv::Mat M = warp.matrix();
  samplePatchAffine(image, patch, M, width, invert, interpolation);
}

void samplePatchAffine(const cv::Mat& src,
                       cv::Mat& dst,
                       const cv::Mat& M,
                       int width,
                       bool invert,
                       int interpolation) {
  cv::Size size(width, width);
  cv::Point2d offset((width - 1) / 2., (width - 1) / 2.);

  // The transformation takes the form dst(x) = src(A x + b).
  // However, we want this to be true for x' = x + c.
  // A' (x + c) + b' = A x + b
  // => A' = A
  //    b' = -A c + b        (by equating co-efficients)
  cv::Mat Q = M.clone();
  cv::Mat c = (cv::Mat_<double>(3, 1) << -offset.x, -offset.y, 1.);
  // Caution: Assignment of MatExpr to matrix of correct size.
  Q.col(2) = M * c;

  int invert_flags;
  if (invert) {
    // Remember our definition of inverted is the opposite to OpenCV's.
    invert_flags = 0;
  } else {
    invert_flags = cv::WARP_INVERSE_MAP;
  }

  int flags = interpolation | invert_flags;

  // Invert the warp. OpenCV doesn't seem to be doing what it says.
  cv::warpAffine(src, dst, Q, size, flags, cv::BORDER_CONSTANT,
      cv::Scalar::all(0.));
  //warpAffine(src, dst, Q, size);
}

#if 0
// Unused. Implemented as a sanity check.
// This is what the OpenCV documentation says it is doing, but I could only
// achieve identical behaviour using the WARP_INVERSE_MAP flag.
void warpAffine(const cv::Mat& src,
                cv::Mat& dst,
                const cv::Mat& M,
                const cv::Size& size) {
  if (src.type() != cv::DataType<double>::type) {
    throw std::runtime_error("expected 64-bit floating point number");
  }

  dst.create(size.height, size.width, src.type());
  dst = cv::Scalar::all(0);

  for (int x = 0; x < size.width; x += 1) {
    for (int y = 0; y < size.height; y += 1) {
      cv::Mat p = (cv::Mat_<double>(3, 1) << x, y, 1);
      cv::Mat q = M * p;
      int u = std::floor(q.at<double>(0, 0) + 0.5);
      int v = std::floor(q.at<double>(1, 0) + 0.5);

      if (u >= 0 && u < src.cols && v >= 0 && v < src.rows) {
        // Non-inverted mapping!
        dst.at<double>(y, x) = src.at<double>(v, u);
      }
    }
  }
}
#endif
