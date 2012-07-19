#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/scoped_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ceres/ceres.h>
#include "read_image.hpp"
#include "track.hpp"
#include "tracker.hpp"
#include "klt_tracker.hpp"

// Size of window to track. Default: 7.
const int WINDOW_SIZE = 31;
const int NUM_PIXELS = WINDOW_SIZE * WINDOW_SIZE;

const int MAX_ITERATIONS = 100;

// Represents a parametrised warp function.
/*
class WarpType {
  public:
    // Evaluate the warp.
    virtual cv::Point2d operator()(const cv::Point2d& x,
                                   const std::vector<double>& params) const = 0;

    // Provides the number of parameters in the warp.
    virtual int numParameters() const = 0;
};
*/

std::string makeFilename(const std::string& format, int n) {
  return boost::str(boost::format(format) % (n + 1));
}

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

void sample(const cv::Mat& src, cv::Mat& dst, const cv::Mat& M, int width) {
  // Size of patch.
  cv::Size size(width, width);
  double offset = (width - 1) / 2.;

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

void translationWarp(cv::Mat& M, double x, double y) {
  M.create(2, 3, cv::DataType<double>::type);
  M.at<double>(0, 0) = 1;
  M.at<double>(0, 1) = 0;
  M.at<double>(0, 2) = x;
  M.at<double>(1, 0) = 0;
  M.at<double>(1, 1) = 1;
  M.at<double>(1, 2) = y;
}

void rigidWarp(cv::Mat& M, double x, double y, double scale, double theta) {
  M.create(2, 3, cv::DataType<double>::type);
  M.at<double>(0, 0) = scale * std::cos(theta);
  M.at<double>(0, 1) = scale * std::sin(theta);
  M.at<double>(0, 2) = x;
  M.at<double>(1, 0) = scale * -std::sin(theta);
  M.at<double>(1, 1) = scale * std::cos(theta);
  M.at<double>(1, 2) = y;
}

class TranslationWarp {
  public:
    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const {
      // x' = x + tx
      // y' = y + ty
      q[0] = x[0] + p[0];
      q[1] = x[1] + p[1];

      return true;
    }
};

class RigidWarp {
  public:
    template<class T>
    bool operator()(const T* const x, const T* const p, T* q) const {
      // x' = scale * ( cos(theta) * x + sin(theta) * y) + tx
      // y' = scale * (-sin(theta) * x + cos(theta) * y) + ty
      q[0] = p[2] * ( cos(p[3]) * x[0] + sin(p[3]) * x[1]) + p[0];
      q[1] = p[2] * (-sin(p[3]) * x[0] + cos(p[3]) * x[1]) + p[1];

      return true;
    }
};

// Translation warp has two parameters (x, y).
class TranslationWarpCost : public ceres::SizedCostFunction<NUM_PIXELS, 2> {
  public:
    TranslationWarpCost(const cv::Mat& J,
                        const cv::Mat& I,
                        const cv::Mat& dIdx,
                        const cv::Mat& dIdy)
        : J_(&J),
          I_(&I),
          dIdx_(&dIdx),
          dIdy_(&dIdy),
          warp_(new TranslationWarp) {}

    ~TranslationWarpCost() {}

    bool Evaluate(double const* const* params,
                  double* residuals,
                  double** jacobians) const {
      // Build affine transform.
      cv::Mat M;
      translationWarp(M, params[0][0], params[0][1]);

      // Sample image for x in regular grid.
      cv::Mat patch;
      sample(*I_, patch, M, WINDOW_SIZE);

      // Compute residuals.
      cv::Mat error = cv::Mat_<double>(WINDOW_SIZE, WINDOW_SIZE, residuals);
      error = patch - *J_;

      if (jacobians != NULL && jacobians[0] != NULL) {
        // f(x, p) = I(W(x, p))
        // df/dp(x, p) = dI/dx(W(x, p)) dW/dp(x, p)

        // Sample whole patches of derivative image.
        // (for efficiency and hopefully correct downsampling)
        cv::Mat ddx_patch;
        cv::Mat ddy_patch;
        sample(*dIdx_, ddx_patch, M, WINDOW_SIZE);
        sample(*dIdy_, ddy_patch, M, WINDOW_SIZE);

        double offset = (WINDOW_SIZE - 1) / 2.;

        for (int u = 0; u < WINDOW_SIZE; u += 1) {
          for (int v = 0; v < WINDOW_SIZE; v += 1) {
            // Get image derivative at each warped point.
            cv::Mat dIdx = (cv::Mat_<double>(1, 2) <<
                ddx_patch.at<double>(v, u), ddy_patch.at<double>(v, u));

            // Get derivative of warp function.
            double position[2] = { u - offset, v - offset };
            const double* warp_params[] = { position, params[0] };
            // Give somewhere to write the warp to.
            double W[2];
            // Need 2x2 matrix with contiguous storage to write out Jacobian.
            double dWdp_data[2 * 2];
            // Only ask for the derivative of the second argument.
            double* warp_jacobians[2] = { NULL, dWdp_data };
            // Use Ceres to get the derivative of the warp function.
            warp_.Evaluate(warp_params, W, warp_jacobians);

            // Use chain rule.
            cv::Mat dWdp(2, 2, cv::DataType<double>::type, dWdp_data);

            // Compute 1x2 Jacobian.
            // Row-major order.
            int i = v * WINDOW_SIZE + u;
            cv::Mat dfdp = cv::Mat_<double>(1, 2, &jacobians[0][i * 2]);
            dfdp = dIdx * dWdp;
          }
        }
      }

      return true;
    }

  private:
    const cv::Mat* J_;
    const cv::Mat* I_;
    const cv::Mat* dIdx_;
    const cv::Mat* dIdy_;
    ceres::AutoDiffCostFunction<TranslationWarp, 2, 2, 2> warp_;
};

// Rigid warp has four parameters (x, y, s, theta).
class RigidWarpCost : public ceres::SizedCostFunction<NUM_PIXELS, 4> {
  public:
    RigidWarpCost(const cv::Mat& J,
                  const cv::Mat& I,
                  const cv::Mat& dIdx,
                  const cv::Mat& dIdy)
        : J_(&J), I_(&I), dIdx_(&dIdx), dIdy_(&dIdy), warp_(new RigidWarp) {}

    ~RigidWarpCost() {}

    bool Evaluate(double const* const* params,
                  double* residuals,
                  double** jacobians) const {
      // Build affine transform.
      cv::Mat M;
      rigidWarp(M, params[0][0], params[0][1], params[0][2], params[0][3]);

      // Sample image for x in regular grid.
      cv::Mat patch;
      sample(*I_, patch, M, WINDOW_SIZE);

      // Compute squared difference.
      cv::Mat error = cv::Mat_<double>(WINDOW_SIZE, WINDOW_SIZE, residuals);
      error = patch - *J_;

      if (jacobians != NULL && jacobians[0] != NULL) {
        // f(x, p) = I(W(x, p))
        // df/dp(x, p) = dI/dx(W(x, p)) dW/dp(x, p)

        // Sample whole patches of derivative image.
        // (for efficiency and hopefully correct downsampling)
        cv::Mat ddx_patch;
        cv::Mat ddy_patch;
        sample(*dIdx_, ddx_patch, M, WINDOW_SIZE);
        sample(*dIdy_, ddy_patch, M, WINDOW_SIZE);

        double offset = (WINDOW_SIZE - 1) / 2.;

        for (int u = 0; u < WINDOW_SIZE; u += 1) {
          for (int v = 0; v < WINDOW_SIZE; v += 1) {
            // Get image derivative at each warped point.
            cv::Mat dIdx = (cv::Mat_<double>(1, 2) <<
                ddx_patch.at<double>(v, u), ddy_patch.at<double>(v, u));

            // Get derivative of warp function.
            double position[2] = { u - offset, v - offset };
            const double* warp_params[] = { position, params[0] };
            // Give somewhere to write the warp to.
            double W[2];
            // Need 2x4 matrix with contiguous storage to write out Jacobian.
            double dWdp_data[2 * 4];
            // Only ask for the derivative of the second argument.
            double* warp_jacobians[2] = { NULL, dWdp_data };
            // Use Ceres to get the derivative of the warp function.
            warp_.Evaluate(warp_params, W, warp_jacobians);

            // Use chain rule.
            cv::Mat dWdp(2, 4, cv::DataType<double>::type, dWdp_data);

            // Compute 1x4 Jacobian.
            // Row-major order.
            int i = v * WINDOW_SIZE + u;
            cv::Mat dfdp = cv::Mat_<double>(1, 4, &jacobians[0][i * 4]);
            dfdp = dIdx * dWdp;
          }
        }
      }

      return true;
    }

  private:
    const cv::Mat* J_;
    const cv::Mat* I_;
    const cv::Mat* dIdx_;
    const cv::Mat* dIdy_;
    ceres::AutoDiffCostFunction<RigidWarp, 2, 2, 4> warp_;
};

/*
void solveFlow(const cv::Mat& image1,
               const cv::Mat& image2,
               const cv::Mat& image1_ddx,
               const cv::Mat& image1_ddy,
               WarpType& warp,
               const std::vector<double>& params1,
               std::vector<double>& params2,
               int patch_size) {
  // Extract reference patch from first image.
  cv::Mat M;
  //warp(params1, M);
  cv::Mat ref;
  sample(image1, ref, M, patch_size);

  // Solve system.
}
*/

struct TranslationFeature {
  double x;
  double y;

  void draw(cv::Mat& image) const {
    cv::Scalar color(0, 0, 255);
    int thickness = 2;

    cv::Point2d c(x, y);
    cv::Point2d i(1, 0);
    cv::Point2d j(0, 1);
    double radius = (WINDOW_SIZE - 1) / 2.;
    i *= radius;
    j *= radius;

    cv::line(image, c - i - j, c + i - j, color, thickness);
    cv::line(image, c - i + j, c + i + j, color, thickness);
    cv::line(image, c - i - j, c - i + j, color, thickness);
    cv::line(image, c + i - j, c + i + j, color, thickness);
  }
};

struct RigidFeature {
  double x;
  double y;
  double scale;
  double theta;

  void draw(cv::Mat& image) const {
    cv::Scalar color(0, 0, 255);
    int thickness = 2;

    cv::Point2d c(x, y);
    cv::Point2d i(std::cos(-theta), std::sin(-theta));
    cv::Point2d j(std::sin(-theta), -std::cos(-theta));

    double radius = scale * (WINDOW_SIZE - 1) / 2;
    i *= radius;
    j *= radius;

    cv::line(image, c - i - j, c + i - j, color, thickness);
    cv::line(image, c - i + j, c + i + j, color, thickness);
    cv::line(image, c - i - j, c - i + j, color, thickness);
    cv::line(image, c + i - j, c + i + j, color, thickness);
    cv::line(image, c, c + j, color, thickness);
  }
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " image-format" << std::endl;
    return 1;
  }

  std::string image_format = argv[1];

  int t = 0;
  bool ok = true;
  cv::Mat previous_image;

  // Start at image center with unit scale and zero orientation.
  RigidFeature feature;

  while (ok) {
    // Read image.
    cv::Mat integer_image;
    cv::Mat color_image;
    ok = readImage(makeFilename(image_format, t), color_image, integer_image);
    if (!ok) {
      std::cerr << "could not read image" << std::endl;
      return 1;
    }

    // Convert to floating point.
    cv::Mat image;
    integer_image.convertTo(image, cv::DataType<double>::type, 1. / 255.);

    if (t == 0) {
      feature.x = 540;
      feature.y = 220;
      feature.scale = 1;
      feature.theta = 0;
    } else {
      // Sample patch from previous image.
      cv::Mat M;
      rigidWarp(M, feature.x, feature.y, feature.scale, feature.theta);
      //translationWarp(M, feature.x, feature.y);
      cv::Mat reference;
      sample(previous_image, reference, M, WINDOW_SIZE);

      // Take x and y derivatives.
      cv::Mat dIdx;
      cv::Mat dIdy;
      cv::Mat diff = (cv::Mat_<double>(1, 3) << -0.5, 0, 0.5);
      cv::Mat identity = (cv::Mat_<double>(1, 1) << 1);
      cv::sepFilter2D(image, dIdx, -1, diff, identity);
      cv::sepFilter2D(image, dIdy, -1, identity, diff);

      // Set up non-linear optimization problem.
      ceres::Problem problem;
      //problem.AddResidualBlock(
      //    new ceres::NumericDiffCostFunction<TranslationWarpCost, ceres::CENTRAL, NUM_PIXELS, 2>(
      //      new TranslationWarpCost(reference, image, dIdx, dIdy),
      //      ceres::TAKE_OWNERSHIP),
      //    NULL, &feature.x);
      problem.AddResidualBlock(new RigidWarpCost(reference, image, dIdx, dIdy),
          NULL, &feature.x);

      ceres::Solver::Options options;
      options.max_num_iterations = MAX_ITERATIONS;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = false;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      std::cout << "(" << feature.x << ", " << feature.y << ")" << std::endl;
      std::cerr << summary.BriefReport() << std::endl;

      /*
      // Do not use Ceres for the actual optimization, only for derivatives.
      TranslationWarpCost warp_cost(reference, image, dIdx, dIdy);

      int n = 0;

      int NUM_PARAMS = 2;

      while (n < MAX_ITERATIONS) {
        // Value of objective function.
        double objective;
        // Need contiguous array.
        double J_data[NUM_PIXELS * NUM_PARAMS];
        // Evaluate.
        const double* params[] = { &feature.x };
        double* jacobians[] = { J_data };
        warp_cost.Evaluate(params, &objective, jacobians);

        cv::Mat J = cv::Mat_<double>(NUM_PIXELS, NUM_PARAMS, J_data);

        std::cout << J << std::endl;
        exit(1);

        n += 1;
      }
      */

      // Visualize.
      feature.draw(color_image);

      cv::imshow("frame", color_image);
      cv::waitKey(10);
    }

    image.copyTo(previous_image);
    t += 1;
  }

  return 0;
}
