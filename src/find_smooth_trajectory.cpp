#include "find_smooth_trajectory.hpp"
#include <deque>
#include <ceres/ceres.h>

const int MAX_NUM_ITERATIONS = 100;
const double FUNCTION_TOLERANCE = 1e-6;
const double GRADIENT_TOLERANCE = 1e-6;
const double PARAMETER_TOLERANCE = 1e-6;

// Using the ceres non-linear solver to solve a sparse linear system.
class AlgebraicError {
  public:
    AlgebraicError(const PointObservation& observation, double lambda);

    template<class T>
    bool operator()(const T* const x, T* r) const;

  private:
    const PointObservation observation_;
    double lambda_;
};

AlgebraicError::AlgebraicError(const PointObservation& observation,
                               double lambda)
    : observation_(observation), lambda_(lambda) {}

template<class T>
bool AlgebraicError::operator()(const T* const x, T* r) const {
  const cv::Matx34d& P = observation_.P;
  const cv::Point2d& w = observation_.w;

  T u = P(0, 0) * x[0] + P(0, 1) * x[1] + P(0, 2) * x[2] + P(0, 3);
  T v = P(1, 0) * x[0] + P(1, 1) * x[1] + P(1, 2) * x[2] + P(1, 3);
  T d = P(2, 0) * x[0] + P(2, 1) * x[1] + P(2, 2) * x[2] + P(2, 3);

  // Geometric error:
  // r[0] = u / d - w.x;
  // r[1] = v / d - w.y;
  // Algebraic error:
  r[0] = lambda_ * (u - w.x * d);
  r[1] = lambda_ * (v - w.y * d);

  return true;
}

////////////////////////////////////////////////////////////////////////////////

class GeometricError {
  public:
    GeometricError(const PointObservation& observation, double lambda);

    template<class T>
    bool operator()(const T* const x, T* r) const;

  private:
    const PointObservation observation_;
    double lambda_;
};

GeometricError::GeometricError(const PointObservation& observation,
                               double lambda)
    : observation_(observation), lambda_(lambda) {}

template<class T>
bool GeometricError::operator()(const T* const x, T* r) const {
  const cv::Matx34d& P = observation_.P;
  const cv::Point2d& w = observation_.w;

  T u = P(0, 0) * x[0] + P(0, 1) * x[1] + P(0, 2) * x[2] + P(0, 3);
  T v = P(1, 0) * x[0] + P(1, 1) * x[1] + P(1, 2) * x[2] + P(1, 3);
  T d = P(2, 0) * x[0] + P(2, 1) * x[1] + P(2, 2) * x[2] + P(2, 3);

  r[0] = lambda_ * (u / d - w.x);
  r[1] = lambda_ * (v / d - w.y);

  return true;
}

////////////////////////////////////////////////////////////////////////////////

class SecondOrderSmoothness {
  public:
    SecondOrderSmoothness(double lambda);

    template<class T>
    bool operator()(const T* const x1,
                    const T* const x2,
                    const T* const x3,
                    T* r) const;

  private:
    double lambda_;
};

SecondOrderSmoothness::SecondOrderSmoothness(double lambda) : lambda_(lambda) {}

template<class T>
bool SecondOrderSmoothness::operator()(const T* const x1,
                                       const T* const x2,
                                       const T* const x3,
                                       T* r) const {
  r[0] = lambda_ * (x1[0] - 2. * x2[0] + x3[0]);
  r[1] = lambda_ * (x1[1] - 2. * x2[1] + x3[1]);
  r[2] = lambda_ * (x1[2] - 2. * x2[2] + x3[2]);

  return true;
}

////////////////////////////////////////////////////////////////////////////////

void addProjectionTerms(ceres::Problem& problem,
                        const MultiviewTrack<PointObservation>& observations,
                        Track<cv::Point3d>& trajectory,
                        bool algebraic,
                        double lambda) {
  // Projection terms.
  MultiviewTrack<PointObservation>::TimeIterator frame(observations);
  Track<cv::Point3d>::iterator point = trajectory.begin();

  while (!frame.end()) {
    // Extract observations at this instant.
    std::map<int, PointObservation> current;
    frame.get(current);

    // Add a term to the residual for each view.
    // Assume that x, y, z in cv::Point3d are laid out contiguously.
    std::map<int, PointObservation>::const_iterator view;
    for (view = current.begin(); view != current.end(); ++view) {
      if (algebraic) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AlgebraicError, 2, 3>(
              new AlgebraicError(view->second, lambda)),
            NULL, &point->second.x);
      } else {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<GeometricError, 2, 3>(
              new GeometricError(view->second, lambda)),
            NULL, &point->second.x);
      }
    }

    frame.next();
  }
}

void addSmoothnessTerms(ceres::Problem& problem,
                        Track<cv::Point3d>& trajectory,
                        double lambda) {
  const size_t FILTER_SUPPORT = 3;

  // Circular buffer of last m points.
  typedef std::deque<Track<cv::Point3d>::iterator> Buffer;
  Buffer support;

  // Initialize buffer.
  Track<cv::Point3d>::iterator point = trajectory.begin();
  bool valid = true;

  // Add first m - 1 elements to buffer.
  while (support.size() < FILTER_SUPPORT - 1 && valid) {
    if (point == trajectory.end()) {
      valid = false;
    } else {
      support.push_back(point);
      ++point;
    }
  }

  while (point != trajectory.end()) {
    support.push_back(point);

    Buffer::iterator p = support.begin();
    cv::Point3d& x1 = (*p++)->second;
    cv::Point3d& x2 = (*p++)->second;
    cv::Point3d& x3 = (*p++)->second;

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SecondOrderSmoothness, 3, 3, 3, 3>(
          new SecondOrderSmoothness(lambda)),
        NULL, &x1.x, &x2.x, &x3.x);

    support.pop_front();
    ++point;
  }
}

void findSmoothTrajectory(const MultiviewTrack<PointObservation>& observations,
                          double lambda,
                          Track<cv::Point3d>& trajectory,
                          double& residual,
                          double& condition) {
  trajectory.clear();

  int a = observations.firstFrameNumber();
  int b = observations.lastFrameNumber();
  trajectory.resetRange(a, b);

  ceres::Problem problem;
  addProjectionTerms(problem, observations, trajectory, true, 1.);
  addSmoothnessTerms(problem, trajectory, lambda);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = MAX_NUM_ITERATIONS;
  options.function_tolerance = FUNCTION_TOLERANCE;
  options.gradient_tolerance = GRADIENT_TOLERANCE;
  options.parameter_tolerance = PARAMETER_TOLERANCE;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
}
