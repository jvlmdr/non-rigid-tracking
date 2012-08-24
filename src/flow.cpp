#include "flow.hpp"
#include <stdexcept>
#include <numeric>
#include <glog/logging.h>
#include <boost/scoped_array.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ceres/ceres.h>
#include "util.hpp"

class WarpCost : public ceres::CostFunction {
  public:
    // Constructs a warp cost function.
    //
    // Parameters:
    // J -- Template patch, must be square.
    // I -- Image within which to find the template patch.
    // dIdx, dIdy -- The x and y derivative of the image.
    // mask -- Weighting mask, same size as template.
    //   Should be rotationally symmetric and circular.
    WarpCost(const Warp& warp,
             const cv::Mat& J,
             const cv::Mat& I,
             const cv::Mat& dIdx,
             const cv::Mat& dIdy,
             const cv::Mat& mask,
             int interpolation,
             bool check_condition,
             double max_condition,
             const WarpValidator* validator)
        : warp_(&warp),
          J_(&J),
          I_(&I),
          dIdx_(&dIdx),
          dIdy_(&dIdy),
          mask_(&mask),
          interpolation_(interpolation),
          check_condition_(check_condition),
          max_condition_(max_condition),
          validator_(validator) {
      // Check that we have a square patch.
      CHECK(J.rows == J.cols);
      // Check that mask is correct size.
      CHECK(mask.size() == J.size());

      // Set the number of inputs (configure as a single block).
      mutable_parameter_block_sizes()->push_back(warp_->numParams());

      // Set the number of outputs.
      set_num_residuals(J.total());
    }

    ~WarpCost() {}

    bool Evaluate(const double* const* params,
                  double* residuals,
                  double** jacobians) const {
      // Check that configuration is valid.
      if (validator_ != NULL) {
        if (!validator_->check(params[0])) {
          return false;
        }
      }

      int width = J_->rows;
      int num_pixels = width * width;
      int num_params = warp_->numParams();

      // Build matrix representing affine transformation.
      cv::Mat M = warp_->matrix(params[0]);
      // Sample image for x in regular grid.
      cv::Mat patch;
      samplePatchAffine(*I_, patch, M, width, false, interpolation_);

      // Compute residuals.
      cv::Mat error = cv::Mat_<double>(width, width, residuals);
      error = patch - *J_;
      // Weight by the mask.
      cv::multiply(error, *mask_, error);

      if (jacobians != NULL && jacobians[0] != NULL) {
        cv::Mat jac = cv::Mat_<double>(num_pixels, num_params, jacobians[0]);

        // f(x, p) = I(W(x, p)) - J(x)
        // df/dp(x, p) = dI/dx(W(x, p)) dW/dp(x, p)

        // g(x, p) = M(x) f(x, p)
        // dg/dp(x, p) = M(x) df/dp(x, p)

        // Sample whole patches of derivative image.
        // (for efficiency and hopefully correct downsampling)
        cv::Mat ddx_patch;
        cv::Mat ddy_patch;
        samplePatchAffine(*dIdx_, ddx_patch, M, width, false, interpolation_);
        samplePatchAffine(*dIdy_, ddy_patch, M, width, false, interpolation_);

        double radius = (width - 1) / 2.;
        cv::Point2d center(radius, radius);

        for (int u = 0; u < width; u += 1) {
          for (int v = 0; v < width; v += 1) {
            // Get row-major order index.
            int i = v * width + u;

            // Get image derivative at each warped point.
            cv::Mat dIdx = (cv::Mat_<double>(1, 2) <<
                ddx_patch.at<double>(v, u), ddy_patch.at<double>(v, u));

            // Get derivative of warp function.
            double dWdp_data[2 * num_params];
            cv::Point2d position = cv::Point2d(u, v) - center;
            warp_->evaluate(position, params[0], dWdp_data);

            // Use chain rule.
            cv::Mat dWdp(2, num_params, cv::DataType<double>::type, dWdp_data);

            // Compute partial Jacobian.
            double m = mask_->at<double>(v, u);
            jac.row(i) = m * dIdx * dWdp;
          }
        }

        if (check_condition_) {
          // Check that Jacobian is well-conditioned.
          double condition = cond(jac);

          if (condition > max_condition_) {
            DLOG(INFO) << "Condition number of Jacobian too large (" <<
              condition << " > " << max_condition_ << ")";
            return false;
          }
        }
      }

      return true;
    }

  private:
    const Warp* warp_;
    const cv::Mat* J_;
    const cv::Mat* I_;
    const cv::Mat* dIdx_;
    const cv::Mat* dIdy_;
    const cv::Mat* mask_;
    int interpolation_;
    bool check_condition_;
    double max_condition_;
    const WarpValidator* validator_;
};

// Returns false if the optimization did not converge.
bool trackPatch(const Warp& warp,
                const cv::Mat& reference,
                const cv::Mat& image,
                const cv::Mat& ddx_image,
                const cv::Mat& ddy_image,
                double* params,
                const cv::Mat& mask,
                const FlowOptions& options,
                const WarpValidator* validator) {
  CHECK(reference.rows == reference.cols) << "Template must be square";

  // Set up non-linear optimization problem.
  ceres::CostFunction* objective = new WarpCost(warp, reference,
      image, ddx_image, ddy_image, mask, options.interpolation,
      options.check_condition, options.max_condition, validator);

  ceres::Problem problem;
  problem.AddResidualBlock(objective, NULL, params);

  // Solve!
  ceres::Solver::Summary summary;
  ceres::Solve(options.solver_options, &problem, &summary);

  // Ensure there was no catastrophic failure.
  CHECK(summary.termination_type != ceres::DID_NOT_RUN);

  // Numerical failure can be caused by e.g. numbers going to infinity.
  if (summary.termination_type == ceres::NUMERICAL_FAILURE) {
    DLOG(INFO) << "Numerical failure";
    return false;
  }

  // Iteration limit was reached before any of the convergece criteria?
  if (options.iteration_limit_is_fatal &&
      summary.termination_type == ceres::NO_CONVERGENCE) {
    DLOG(INFO) << "Reached iteration limit";
    return false;
  }

  return true;
}
