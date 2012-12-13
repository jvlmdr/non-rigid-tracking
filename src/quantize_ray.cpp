#include "quantize_ray.hpp"
#include <set>
#include <numeric>
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <boost/math/tools/roots.hpp>
#include "distortion.hpp"
#include "util.hpp"

double EPSILON = 1e-6;

double errorInDistanceFromPoint(cv::Point2d y,
                                double lambda,
                                const cv::Mat A,
                                const cv::Mat B,
                                double delta,
                                const CameraProperties& intrinsics) {
  cv::Mat X = A + lambda * B;
  cv::Point2d x = imagePointFromHomogeneous(X);
  x = intrinsics.distortAndUncalibrate(x);
  return cv::norm(x - y) - delta;
}

struct ProjectiveLine {
  cv::Mat A;
  cv::Mat B;
};

// Everything you need to quantize a 3D ray in one view.
struct Quantizer {
  // Index for establishing uniqueness and order in a set.
  int index;
  // Parameters of 2D line (calibrated and undistorted).
  ProjectiveLine line;
  // Interval of ray which is in front of camera.
  double lambda_min;
  double lambda_max;
  // Position of current lambda.
  // TODO: This should be a map not a set!
  mutable cv::Point2d x;

  CameraProperties intrinsics;

  bool operator<(const Quantizer& other) const {
    return index < other.index;
  }
};

double maximumErrorInDistanceFromPoint(const std::set<Quantizer>& views,
                                       double lambda,
                                       double delta) {
  double max = -std::numeric_limits<double>::infinity();

  std::set<Quantizer>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view) {
    CHECK(view->lambda_min <= lambda);
    CHECK(lambda <= view->lambda_max);

    // Compute position on line.
    cv::Mat X = view->line.A + lambda * view->line.B;
    cv::Point2d x = imagePointFromHomogeneous(X);
    x = view->intrinsics.distortAndUncalibrate(x);

    // Compute error in distance from previous position.
    double e = cv::norm(x - view->x) - delta;

    if (e > max) {
      max = e;
    }
  }

  return max;
}

double maxLambdaMin(double lambda_min, const Quantizer& quantizer) {
  return std::max(lambda_min, quantizer.lambda_min);
}

bool lambdaMinGreaterThan(const Quantizer& quantizer, double lambda) {
  return quantizer.lambda_min > lambda;
}

double computeLambdaMax(const cv::Mat& A,
                        const cv::Mat& B,
                        const CameraProperties& intrinsics,
                        cv::Point2d& x) {
  double a3 = A.at<double>(2, 0);
  double b3 = B.at<double>(2, 0);

  double lambda_max;

  // Set lambda_max and lambda depending on destination of ray.
  if (b3 < 0) {
    // Ray goes to infinity in front of camera. There is a vanishing point.
    DLOG(INFO) << "Ray ends in front of camera";
    // No upper bound on lambda.
    lambda_max = std::numeric_limits<double>::infinity();

    // Compute vanishing point.
    x = imagePointFromHomogeneous(B);
    x = intrinsics.distortAndUncalibrate(x);
  } else {
    // Ray goes to infinity behind camera, crossing image plane. There is no
    // vanishing point. However, under distortion, a 2D point at infinity
    // will still have a finite position.
    DLOG(INFO) << "Ray ends behind camera";
    lambda_max = -a3 / b3;
    CHECK(lambda_max > 0);

    // Find position of point at infinity after distortion.
    cv::Mat X = A + lambda_max * B;
    x = cv::Point2d(X.at<double>(0, 0), X.at<double>(1, 0));
    x = distortPointAtInfinity(x, intrinsics.distort_w);
    x = intrinsics.uncalibrate(x);

    // Shrink by some epsilon to allow non-strict inequality.
    lambda_max *= (1. - EPSILON);
  }

  return lambda_max;
}

double computeLambdaMin(const cv::Mat& A,
                        const cv::Mat& B,
                        const CameraProperties& intrinsics,
                        cv::Point2d& x) {
  double a3 = A.at<double>(2, 0);
  double b3 = B.at<double>(2, 0);

  double lambda_min;

  // Set lambda_min depending on position of camera center.
  if (a3 < 0) {
    // Ray starts in front of camera. 2D line starts at a finite coordinate.
    DLOG(INFO) << "Ray starts in front of camera";
    lambda_min = 0;

    // Compute epipole.
    x = imagePointFromHomogeneous(A);
    x = intrinsics.distortAndUncalibrate(x);
  } else {
    // Ray starts behind camera. 2D line starts at infinity.
    DLOG(INFO) << "Ray starts behind camera";
    lambda_min = -a3 / b3;
    CHECK(lambda_min > 0);

    // Find position of point at infinity after distortion.
    cv::Mat X = A + lambda_min * B;
    x = cv::Point2d(X.at<double>(0, 0), X.at<double>(1, 0));
    x = distortPointAtInfinity(x, intrinsics.distort_w);
    x = intrinsics.uncalibrate(x);

    // Grow by some epsilon to allow non-strict inequality.
    lambda_min *= (1. + EPSILON);
  }

  return lambda_min;
}

void quantizeRay(const cv::Point2d& projection,
                 const std::vector<Camera>& cameras,
                 int selected,
                 double delta,
                 std::map<double, cv::Point3d>& points) {
  points.clear();

  // Solutions parametrized by 3D line c + lambda v, lambda >= 0.
  const Camera& camera = cameras[selected];
  cv::Point3d c = camera.extrinsics().center;
  cv::Point2d w = camera.intrinsics().calibrateAndUndistort(projection);
  cv::Point3d v = camera.extrinsics().directionOfRayThrough(w);

  cv::Mat C = worldPointToHomogeneous(c);
  cv::Mat V = worldPointToHomogeneous(v, 0);

  // Initialize Quantizer for each view.
  std::set<Quantizer> pending;

  {
    std::vector<Camera>::const_iterator other;
    int index = 0;

    for (other = cameras.begin(); other != cameras.end(); ++other) {
      if (index != selected) {
        // Find 2D projective line.
        cv::Mat P(other->extrinsics().matrix());
        ProjectiveLine line;
        line.A = P * C;
        line.B = P * V;

        // Check whether each point is in front of or behind the camera.
        double a3 = line.A.at<double>(2, 0);
        double b3 = line.B.at<double>(2, 0);

        if (a3 > 0 && b3 > 0) {
          // Entire ray is behind camera.
          DLOG(INFO) << "Ray is not observed";
        } else {
          double lambda_min;
          double lambda_max;
          cv::Point2d x;

          lambda_max = computeLambdaMax(line.A, line.B, other->intrinsics(), x);
          cv::Point2d y;
          lambda_min = computeLambdaMin(line.A, line.B, other->intrinsics(), y);

          Quantizer view;
          view.index = index;
          view.line = line;
          view.lambda_min = lambda_min;
          view.lambda_max = lambda_max;
          view.x = x;
          view.intrinsics = other->intrinsics();

          pending.insert(view);
        }
      }

      index += 1;
    }
  }

  // Find initial lambda.
  double lambda = 0;

  for (std::set<Quantizer>::const_iterator view = pending.begin();
       view != pending.end();
       ++view) {
    if (view->lambda_max < std::numeric_limits<double>::infinity()) {
      // lambda_max is finite.
      lambda = std::max(lambda, view->lambda_max);
    } else {
      // lambda_max is infinite.
      // Find a lambda which is big enough.
      bool big_enough = false;

      // Ensure that we don't evaluate below lambda_min.
      lambda = std::max(lambda, view->lambda_min);

      while (!big_enough) {
        double error = errorInDistanceFromPoint(view->x, lambda, view->line.A,
            view->line.B, delta, view->intrinsics);

        if (error < 0) {
          big_enough = true;
        } else {
          if (lambda == 0) {
            lambda = std::max(lambda, 1.);
          } else {
            lambda *= 2;
          }
        }
      }
    }
  }

  bool converged = false;
  int t = 0;

  std::set<Quantizer> active;

  while (!converged) {
    // Move points from pending to active set for which lambda <= lambda_max.
    {
      std::set<Quantizer>::iterator view = pending.begin();
      while (view != pending.end()) {
        // Is the domain of the function within the bisection range?
        if (lambda <= view->lambda_max) {
          active.insert(*view);
          pending.erase(view++);
        } else {
          ++view;
        }
      }
    }

    // Remove points from active set for which lambda < lambda_min.
    {
      std::set<Quantizer> valid;
      std::remove_copy_if(active.begin(), active.end(),
          std::inserter(valid, valid.begin()),
          boost::bind(lambdaMinGreaterThan, _1, lambda));
      active.swap(valid);
    }

    CHECK(!(active.empty() && !pending.empty()));

    // Update positions.
    std::set<Quantizer>::iterator view;
    for (view = active.begin(); view != active.end(); ++view) {
      cv::Mat X = view->line.A + lambda * view->line.B;
      cv::Point2d x = imagePointFromHomogeneous(X);
      x = view->intrinsics.distortAndUncalibrate(x);
      view->x = x;
    }

    // Remove points which do not have a solution in (lambda_min(i), lambda).
    {
      std::set<Quantizer>::iterator view = active.begin();
      while (view != active.end()) {
        // Is there a sign change across the interval?
        double f_max = errorInDistanceFromPoint(view->x, view->lambda_min,
            view->line.A, view->line.B, delta, view->intrinsics);

        // We know that f(lambda_max) < 0, so f(lower) should be > 0.
        if (f_max < 0) {
          active.erase(view++);
        } else {
          ++view;
        }
      }
    }

    if (active.empty() && pending.empty()) {
      converged = true;
    } else {
      CHECK(!active.empty());

      // Update lower bound to be maximum lambda_min over active set.
      double lower = 0;
      lower = std::accumulate(active.begin(), active.end(), lower, maxLambdaMin);

      // Find lambda which gives point at most delta pixels away from x.
      double old_lambda = lambda;
      std::pair<double, double> interval = boost::math::tools::bisect(
          boost::bind(maximumErrorInDistanceFromPoint, active, _1, delta),
          lower, lambda, boost::math::tools::eps_tolerance<double>(16));
      lambda = interval.second;

      // Guard against limit cycles.
      if (t > 0) {
        CHECK(lambda != old_lambda) << "Encountered limit cycle";
      }

      // Add points to tracks.
      points[lambda] = c + lambda * v;
      t += 1;
    }
  }

  DLOG(INFO) << "Quantized ray into " << points.size() << " positions";
}
