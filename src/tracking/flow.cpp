#include "tracking/flow.hpp"
#include <numeric>
#include <glog/logging.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ceres/ceres.h>
#include "util/cond.hpp"
#include "tracking/warper.hpp"

namespace tracking {

namespace {

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
    WarpCost(const Warper& warper,
             const cv::Mat& J,
             const cv::Mat& I,
             const cv::Mat& dIdx,
             const cv::Mat& dIdy,
             const cv::Mat& mask,
             int interpolation,
             bool check_condition,
             double max_condition);

    ~WarpCost() {}

    bool Evaluate(const double* const* params,
                  double* residuals,
                  double** jacobians) const;

  private:
    const Warper* warper_;
    const cv::Mat* J_;
    const cv::Mat* I_;
    const cv::Mat* dIdx_;
    const cv::Mat* dIdy_;
    const cv::Mat* mask_;
    int interpolation_;
    bool check_condition_;
    double max_condition_;
};

WarpCost::WarpCost(const Warper& warper,
                   const cv::Mat& J,
                   const cv::Mat& I,
                   const cv::Mat& dIdx,
                   const cv::Mat& dIdy,
                   const cv::Mat& mask,
                   int interpolation,
                   bool check_condition,
                   double max_condition)
    : warper_(&warper),
      J_(&J),
      I_(&I),
      dIdx_(&dIdx),
      dIdy_(&dIdy),
      mask_(&mask),
      interpolation_(interpolation),
      check_condition_(check_condition),
      max_condition_(max_condition) {
  // Check that we have a square patch.
  CHECK(J.rows == J.cols);
  // Check that mask is correct size.
  CHECK(mask.size() == J.size());

  // Set the number of inputs (configure as a single block).
  mutable_parameter_block_sizes()->push_back(warper_->numParams());

  // Set the number of outputs.
  set_num_residuals(J.total());
}

bool WarpCost::Evaluate(const double* const* params,
                        double* residuals,
                        double** jacobians) const {
  int num_params = warper_->numParams();
  int diameter = J_->rows;
  int radius = (diameter - 1) / 2;

  // Check that configuration is valid.
  if (!warper_->isValid(params[0], I_->size(), radius)) {
    return false;
  }

  int num_pixels = diameter * diameter;

  // Build matrix representing affine transformation.
  cv::Mat M = warper_->matrix(params[0]);
  // Sample image for x in regular grid.
  cv::Mat patch;
  samplePatchAffine(*I_, patch, M, diameter, false, interpolation_);

  // Compute residuals.
  cv::Mat error = cv::Mat_<double>(diameter, diameter, residuals);
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
    samplePatchAffine(*dIdx_, ddx_patch, M, diameter, false, interpolation_);
    samplePatchAffine(*dIdy_, ddy_patch, M, diameter, false, interpolation_);

    cv::Point2d center(radius, radius);

    for (int u = 0; u < diameter; u += 1) {
      for (int v = 0; v < diameter; v += 1) {
        // Get row-major order index.
        int i = v * diameter + u;

        // Get image derivative at each warped point.
        cv::Mat dIdx = (cv::Mat_<double>(1, 2) <<
            ddx_patch.at<double>(v, u), ddy_patch.at<double>(v, u));

        // Get derivative of warp function.
        double dWdp_data[2 * num_params];
        cv::Point2d position = cv::Point2d(u, v) - center;
        warper_->evaluate(position, params[0], dWdp_data);

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

} // namespace

////////////////////////////////////////////////////////////////////////////////

// Returns false if the optimization did not converge.
bool trackPatch(Warp& warp,
                const cv::Mat& reference,
                const cv::Mat& image,
                const cv::Mat& ddx_image,
                const cv::Mat& ddy_image,
                const cv::Mat& mask,
                const FlowOptions& options) {
  CHECK(reference.rows == reference.cols) << "Template must be square";

  // Set up non-linear optimization problem.
  scoped_ptr<Warper> warper(warp.newWarper());
  ceres::CostFunction* objective = new WarpCost(*warper, reference, image,
      ddx_image, ddy_image, mask, options.interpolation,
      options.check_condition, options.max_condition);

  ceres::Problem problem;
  problem.AddResidualBlock(objective, NULL, warp.params());

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

}
