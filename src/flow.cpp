#include "flow.hpp"
#include <stdexcept>
#include <numeric>
#include <boost/scoped_array.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ceres/ceres.h>

////////////////////////////////////////////////////////////////////////////////
// solveFlow

namespace {

class WarpCost : public ceres::CostFunction {
  public:
    WarpCost(const Warp& warp,
             int patch_size,
             const cv::Mat& J,
             const cv::Mat& I,
             const cv::Mat& dIdx,
             const cv::Mat& dIdy)
        : warp_(&warp),
          patch_size_(patch_size),
          J_(&J),
          I_(&I),
          dIdx_(&dIdx),
          dIdy_(&dIdy) {
      int num_pixels = patch_size_ * patch_size_;
      set_num_residuals(num_pixels);
      mutable_parameter_block_sizes()->push_back(warp_->numParams());
    }

    ~WarpCost() {}

    bool Evaluate(const double* const* params,
                  double* residuals,
                  double** jacobians) const {
      // Build affine transform.
      cv::Mat M = warp_->matrix(params[0]);
      // Sample image for x in regular grid.
      cv::Mat patch;
      sampleAffinePatch(*I_, patch, M, patch_size_);

      // Compute residuals.
      cv::Mat error = cv::Mat_<double>(patch_size_, patch_size_, residuals);
      int num_pixels = patch_size_ * patch_size_;
      error = (patch - *J_) / num_pixels;

      if (jacobians != NULL && jacobians[0] != NULL) {
        // f(x, p) = I(W(x, p))
        // df/dp(x, p) = dI/dx(W(x, p)) dW/dp(x, p)

        // Sample whole patches of derivative image.
        // (for efficiency and hopefully correct downsampling)
        cv::Mat ddx_patch;
        cv::Mat ddy_patch;
        sampleAffinePatch(*dIdx_, ddx_patch, M, patch_size_);
        sampleAffinePatch(*dIdy_, ddy_patch, M, patch_size_);

        double offset = (patch_size_ - 1) / 2.;

        for (int u = 0; u < patch_size_; u += 1) {
          for (int v = 0; v < patch_size_; v += 1) {
            // Get image derivative at each warped point.
            cv::Mat dIdx = (cv::Mat_<double>(1, 2) <<
                ddx_patch.at<double>(v, u), ddy_patch.at<double>(v, u));

            // Get derivative of warp function.
            double dWdp_data[2 * warp_->numParams()];
            cv::Point2d position(u - offset, v - offset);
            warp_->evaluate(position, params[0], dWdp_data);

            // Use chain rule.
            cv::Mat dWdp(2, warp_->numParams(), cv::DataType<double>::type,
                dWdp_data);

            // Compute partial Jacobian. Stored in row-major order.
            int i = v * patch_size_ + u;
            cv::Mat dfdp = cv::Mat_<double>(1, warp_->numParams(),
                &jacobians[0][i * warp_->numParams()]);
            dfdp = dIdx * dWdp / num_pixels;
          }
        }
      }

      return true;
    }

  private:
    const Warp* warp_;
    int patch_size_;
    const cv::Mat* J_;
    const cv::Mat* I_;
    const cv::Mat* dIdx_;
    const cv::Mat* dIdy_;
};

}

double cond(const cv::Mat& A) {
  // Compute condition number.
  std::vector<double> sigma;
  cv::SVD::compute(A, sigma, cv::SVD::NO_UV);
  return sigma.front() / sigma.back();
}

// Returns false if the optimisation did not converge.
bool solveFlow(const Warp& warp,
               int patch_size,
               const cv::Mat& reference,
               const cv::Mat& image,
               const cv::Mat& ddx_image,
               const cv::Mat& ddy_image,
               double* params,
               const ceres::Solver::Options& options,
               bool iteration_limit_is_fatal,
               double max_condition) {
  // Set up non-linear optimization problem.
  ceres::CostFunction* objective = new WarpCost(warp, patch_size, reference,
      image, ddx_image, ddy_image);

  ceres::Problem problem;
  problem.AddResidualBlock(objective, NULL, params);

  // Solve!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Get Jacobian at solution and check condition.
  int num_pixels = patch_size * patch_size;
  boost::scoped_array<double> jacobian_data(
      new double[num_pixels * warp.numParams()]);
  cv::Mat jacobian = cv::Mat_<double>(num_pixels, warp.numParams(),
      jacobian_data.get());

  boost::scoped_array<double> residuals(new double[num_pixels]);
  double* parameter_blocks[] = { params };
  double* jacobian_blocks[] = { jacobian_data.get() };
  objective->Evaluate(parameter_blocks, residuals.get(), jacobian_blocks);

  // Check condition of Jacobian at solution.
  // Should really check it at every iteration.
  double condition = cond(jacobian);
  if (condition > max_condition) {
    std::cerr << "condition was over threshold" << std::endl;
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// sampleAffinePatch

namespace {

// Unused. Implemented as a sanity check.
// This is what the OpenCV documentation says it is doing, but I could only
// achieve identical behaviour using the WARP_INVERSE_MAP flag.
void warpAffine(const cv::Mat& src,
                cv::Mat& dst,
                const cv::Mat& M,
                const cv::Size& size) {
  if (src.type() != cv::DataType<uint8_t>::type) {
    throw std::runtime_error("expected unsigned 8-bit integer");
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
        dst.at<uint8_t>(y, x) = src.at<uint8_t>(v, u);
      }
    }
  }
}

}

void sampleAffinePatch(const cv::Mat& src,
                       cv::Mat& dst,
                       const cv::Mat& M,
                       int patch_size) {
  // Size of patch.
  cv::Size size(patch_size, patch_size);
  double offset = (patch_size - 1) / 2.;

  // The transformation takes the form dst(x) = src(A x + b).
  // However, we want this to be true for x' = x + c.
  // A' (x + c) + b' = A x + b
  // => A' = A
  //    b' = -A c + b        (by equating co-efficients)
  cv::Mat Q = M.clone();
  cv::Mat c = (cv::Mat_<double>(3, 1) << -offset, -offset, 1.);
  // Caution: Assignment of MatExpr to matrix of correct size.
  Q.col(2) = M * c;

  // Invert the warp. OpenCV doesn't seem to be doing what it says.
  cv::warpAffine(src, dst, Q, size, cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT, cv::Scalar::all(0.));
  //warpAffine(src, dst, Q, size);
}

////////////////////////////////////////////////////////////////////////////////
// checkAppearance

double meanPixelDifference(const Warp& warp,
                           int patch_size,
                           const cv::Mat& reference,
                           const cv::Mat& image,
                           const double* params) {
  // Sample patch from image.
  cv::Mat M = warp.matrix(params);

  cv::Mat patch;
  sampleAffinePatch(image, patch, M, patch_size);

  cv::Mat diff = cv::abs(reference - patch);
  int num_pixels = patch_size * patch_size;

  return std::accumulate(diff.begin<double>(), diff.end<double>(), 0.) /
      num_pixels;
}

