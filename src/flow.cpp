#include "flow.hpp"
#include <stdexcept>
#include <numeric>
#include <boost/scoped_array.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ceres/ceres.h>

////////////////////////////////////////////////////////////////////////////////
// trackPatch

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
      sampleAffinePatch(*I_, patch, M, patch_size_, false);

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
        sampleAffinePatch(*dIdx_, ddx_patch, M, patch_size_, false);
        sampleAffinePatch(*dIdy_, ddy_patch, M, patch_size_, false);

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
bool trackPatch(const Warp& warp,
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

WarpTracker::WarpTracker(const Warp& warp,
                         int patch_size,
                         int max_num_iterations,
                         double function_tolerance,
                         double gradient_tolerance,
                         double parameter_tolerance,
                         bool iteration_limit_is_fatal,
                         double max_condition)
      : warp_(&warp),
        patch_size_(patch_size),
        options_(),
        iteration_limit_is_fatal_(iteration_limit_is_fatal),
        max_condition_(max_condition) {
  options_.linear_solver_type = ceres::DENSE_QR;
  options_.max_num_iterations = max_num_iterations;
  options_.function_tolerance = function_tolerance;
  options_.gradient_tolerance = gradient_tolerance;
  options_.parameter_tolerance = parameter_tolerance;
  options_.minimizer_progress_to_stdout = true;
}

void WarpTracker::feedImage(const cv::Mat& image) {
  // Rotate buffer and copy new image in.
  previous_image_ = image_;
  image_ = image.clone();

  // Compute gradients.
  cv::Mat diff = (cv::Mat_<double>(1, 3) << -0.5, 0, 0.5);
  cv::Mat identity = (cv::Mat_<double>(1, 1) << 1);
  cv::sepFilter2D(image_, ddx_image_, -1, diff, identity);
  cv::sepFilter2D(image_, ddy_image_, -1, identity, diff);
}

bool WarpTracker::track(double* feature) const {
  // Sample patch from previous image.
  cv::Mat M = warp_->matrix(feature);
  cv::Mat reference;
  sampleAffinePatch(previous_image_, reference, M, patch_size_, false);

  bool ok = trackPatch(*warp_, patch_size_, reference, image_, ddx_image_,
      ddy_image_, feature, options_, iteration_limit_is_fatal_, max_condition_);

  return ok;
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

}

// Returns the number of real solutions (0, 1 or 2).
// Sets v1 to be the larger-magnitude eigenvalue, v2 the second.
int twoByTwoEigenvalues(const cv::Mat& A, double* v1, double* v2) {
  double a = A.at<double>(0, 0);
  double b = A.at<double>(0, 1);
  double c = A.at<double>(1, 0);
  double d = A.at<double>(1, 1);

  // http://en.wikipedia.org/wiki/Eigenvalue_algorithm
  double tr = a + d;
  double det = a * d - b * c;
  double delta = tr * tr - 4 * det;

  int num_real_solutions;

  if (delta < 0) {
    num_real_solutions = 0;
  } else {
    double sqrt_delta = std::sqrt(delta);
    double q1 = (tr + sqrt_delta) / 2;
    double q2 = (tr - sqrt_delta) / 2;

    // The larger magnitude eigenvalue should be the first.
    if (std::abs(q1) < std::abs(q2)) {
      std::swap(q1, q2);
    }

    if (v1 != NULL) {
      *v1 = q1;
    }
    if (v2 != NULL) {
      *v2 = q2;
    }

    if (delta == 0) {
      num_real_solutions = 1;
    } else {
      num_real_solutions = 2;
    }
  }

  return num_real_solutions;
}

void sampleAffinePatch(const cv::Mat& src,
                       cv::Mat& dst,
                       const cv::Mat& M,
                       int patch_width,
                       bool invert) {
  cv::Size size(patch_width, patch_width);
  sampleAffine(src, dst, M, size, invert);
}

void sampleAffine(const cv::Mat& src,
                  cv::Mat& dst,
                  const cv::Mat& M,
                  const cv::Size& size,
                  bool invert) {
  // Size of patch.
  cv::Point2d offset((size.width - 1) / 2., (size.height - 1) / 2.);

  // The transformation takes the form dst(x) = src(A x + b).
  // However, we want this to be true for x' = x + c.
  // A' (x + c) + b' = A x + b
  // => A' = A
  //    b' = -A c + b        (by equating co-efficients)
  cv::Mat Q = M.clone();
  cv::Mat c = (cv::Mat_<double>(3, 1) << -offset.x, -offset.y, 1.);
  // Caution: Assignment of MatExpr to matrix of correct size.
  Q.col(2) = M * c;

  // Get the larger eigenvalue of the rotation/scale component.
  double lambda1;
  twoByTwoEigenvalues(M.colRange(0, 2), &lambda1, NULL);

  // If we are downsampling, use "area" interpolation.
  int interpolation_flags;
  if (lambda1 > 2) {
    interpolation_flags = cv::INTER_AREA;
  } else {
    interpolation_flags = cv::INTER_LINEAR;
  }

  int invert_flags;
  if (invert) {
    // Remember our definition of inverted is the opposite to OpenCV's.
    invert_flags = 0;
  } else {
    invert_flags = cv::WARP_INVERSE_MAP;
  }

  int flags = interpolation_flags | invert_flags;

  // Invert the warp. OpenCV doesn't seem to be doing what it says.
  cv::warpAffine(src, dst, Q, size, flags, cv::BORDER_CONSTANT,
      cv::Scalar::all(0.));
  //warpAffine(src, dst, Q, size);
}

////////////////////////////////////////////////////////////////////////////////
// checkAppearance

double averageResidual(const cv::Mat& A, const cv::Mat& B) {
  cv::Mat diff = cv::abs(A - B);
  int num_pixels = A.total();

  double error = std::accumulate(diff.begin<double>(), diff.end<double>(), 0.);

  return error / num_pixels;
}
