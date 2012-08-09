#include <vector>
#include <complex>
#include <glog/logging.h>
#include "roots.hpp"
#include "util.hpp"

cv::Mat imageTranslationMatrix(const cv::Point2d& p) {
  cv::Mat T = cv::Mat_<double>::eye(3, 3);

  T.at<double>(0, 2) = p.x;
  T.at<double>(1, 2) = p.y;

  return T;
}

// x2^T F x1 = 0
void computeEpipoles(const cv::Mat& F, cv::Mat& e1, cv::Mat& e2) {
  cv::SVD svd(F);
  LOG(INFO) << "Singular values of F: " << svd.w;
  e1 = svd.vt.row(2).t();
  e2 = svd.u.col(2);
}

struct FundMatParams {
  double f1;
  double f2;
  double a;
  double b;
  double c;
  double d;
};

cv::Mat buildFundMatFromParams(const FundMatParams& params) {
  // Maybe this function should go inside FundMatParams purely to avoid this?
  // Although it's not clear where functions using both parametrizations go.
  const double& f1 = params.f1;
  const double& f2 = params.f2;
  const double& a = params.a;
  const double& b = params.b;
  const double& c = params.c;
  const double& d = params.d;

  cv::Mat F = (cv::Mat_<double>(3, 3) <<
      f1 * f2 * d, -f2 * c, -f2 * d,
          -f1 * b,       a,       b,
          -f1 * d,       c,       d);

  return F;
}

double triangulationObjective(double t, const FundMatParams& fund_params) {
  const double& f1 = fund_params.f1;
  const double& f2 = fund_params.f2;
  const double& a = fund_params.a;
  const double& b = fund_params.b;
  const double& c = fund_params.c;
  const double& d = fund_params.d;

  double s = sqr(t) / (1. + sqr(f1 * t)) +
      sqr(c * t + d) / (sqr(a * t + b) + sqr(f2 * (c * t + d)));

  return s;
}

void triangulationPolynomial(const FundMatParams& params,
                             std::vector<double>& p) {
  const double& f1 = params.f1;
  const double& f2 = params.f2;
  const double& a = params.a;
  const double& b = params.b;
  const double& c = params.c;
  const double& d = params.d;

  // Generated using the Matlab Symbolic Toolbox
  // >> syms t f1 f2 a b c d;
  // >> g = t * ((a * t + b)^2 + f2^2 * (c * t + d)^2)^2 -
  //        (a * d - b * c) * (1 + (t * f1)^2)^2 * (a * t + b) * (c * t + d);
  // >> h = coeffs(g, t);
  // >> ccode(h)

  const int DEGREE = 6;
  p.assign(DEGREE, 0);

  p[0] = -b*d*(a*d-b*c);
  p[1] = sqr(b*b+(d*d)*(f2*f2))-a*d*(a*d-b*c)-b*c*(a*d-b*c);
  p[2] = (b*b+(d*d)*(f2*f2))*(a*b*2.0+c*d*(f2*f2)*2.0)*2.0-a*c*(a*d-b*c)-b*d*(f1*f1)*(a*d-b*c)*2.0;
  p[3] = (a*a+(c*c)*(f2*f2))*(b*b+(d*d)*(f2*f2))*2.0+sqr(a*b*2.0+c*d*(f2*f2)*2.0)-a*d*(f1*f1)*(a*d-b*c)*2.0-b*c*(f1*f1)*(a*d-b*c)*2.0;
  p[4] = (a*a+(c*c)*(f2*f2))*(a*b*2.0+c*d*(f2*f2)*2.0)*2.0-a*c*(f1*f1)*(a*d-b*c)*2.0-b*d*(f1*f1*f1*f1)*(a*d-b*c);
  p[5] = sqr(a*a+(c*c)*(f2*f2))-a*d*(f1*f1*f1*f1)*(a*d-b*c)-b*c*(f1*f1*f1*f1)*(a*d-b*c);
  p[6] = -a*c*(f1*f1*f1*f1)*(a*d-b*c);
}

// Returns the transforms to take F to the canonical form for triangulation.
// A2^T F A1
void transformToCanonicalForm(cv::Mat& F,
                              FundMatParams& p,
                              const cv::Point2d& x1,
                              const cv::Point2d& x2,
                              cv::Mat& A1,
                              cv::Mat& A2) {
  // Shift points to the origin.
  cv::Mat T1_inv = imageTranslationMatrix(x1);
  cv::Mat T2_inv = imageTranslationMatrix(x2);

  // Transform the fundamental matrix in this way.
  F = T2_inv.t() * F * T1_inv;

  // Find epipoles.
  cv::Mat e1;
  cv::Mat e2;
  computeEpipoles(F, e1, e2);

  // Normalize.
  e1 /= std::sqrt(sqr(e1.at<double>(0)) + sqr(e1.at<double>(1)));
  e2 /= std::sqrt(sqr(e2.at<double>(0)) + sqr(e2.at<double>(1)));

  // Form rotation matrices to bring to canonical form.
  cv::Mat R1 = (cv::Mat_<double>(3, 3) <<
       e1.at<double>(0), e1.at<double>(1), 0,
      -e1.at<double>(1), e1.at<double>(0), 0,
                      0,                0, 1);
  cv::Mat R2 = (cv::Mat_<double>(3, 3) <<
       e2.at<double>(0), e2.at<double>(1), 0,
      -e2.at<double>(1), e2.at<double>(0), 0,
                      0,                0, 1);
  F = R2 * F * R1.t();

  // Output overall transforms.
  A1 = T1_inv * R1.t();
  A2 = T2_inv * R2.t();

  // Extract parametrization.
  p.f1 = e1.at<double>(2);
  p.f2 = e2.at<double>(2);
  p.a = F.at<double>(1, 1);
  p.b = F.at<double>(1, 2);
  p.c = F.at<double>(2, 1);
  p.d = F.at<double>(2, 2);
}

// Constructs the 2D projections (x1(t), x2(t)) in homogeneous coords.
void buildParametrizedProjections(double t,
                                  cv::Mat& X1,
                                  cv::Mat& X2,
                                  const FundMatParams& p) {
  // Parametrized lines.
  double lambda1[3] = { t * p.f1, 1, -t };
  double lambda2[3] = { -p.f2 * (p.c * t + p.d), p.a * t + p.b, p.c * t + p.d };

  X1 = (cv::Mat_<double>(3, 1) <<
      -lambda1[0] * lambda1[2],
      -lambda1[1] * lambda1[2],
      (sqr(lambda1[0]) + sqr(lambda1[1])));
  X2 = (cv::Mat_<double>(3, 1) <<
      -lambda2[0] * lambda2[2],
      -lambda2[1] * lambda2[2],
      (sqr(lambda2[0]) + sqr(lambda2[1])));
}

double findMinimumRealSolution(
    const std::vector<std::complex<double> >& solutions,
    const FundMatParams& fund_params) {
  int n = solutions.size();
  double min = 0;
  int arg = 0;

  for (int i = 0; i < n; i += 1) {
    // Only consider real part (since all solutions are minima or maxima).
    double t = solutions[i].real();
    double s = triangulationObjective(t, fund_params);

    if (i == 0 || s < min) {
      min = s;
      arg = i;
    }
  }

  if (solutions[arg].imag() != 0) {
    LOG(WARNING) << "Taking real part of complex solution: " << solutions[arg];
  }

  return solutions[arg].real();
}

// Returns the sum of the two residuals.
// Hartley and Sturm, "Triangulation", CVIU 1997.
// Hartley and Zisserman, 2nd ed. p318.
//
// Convention used is x2^T F x1 = 0.
// "First" image co-ordinates multiplied on the right.
double optimalTriangulation(cv::Point2d& x1,
                            cv::Point2d& x2,
                            const cv::Mat& F) {
  // Transform fundamental matrix to canonical form.
  cv::Mat G = F.clone();
  FundMatParams fund_params;
  cv::Mat A1;
  cv::Mat A2;
  transformToCanonicalForm(G, fund_params, x1, x2, A1, A2);

  // Set up polynomial problem to solve for parametrization of points.
  std::vector<double> coeffs;
  triangulationPolynomial(fund_params, coeffs);

  // Solve for all roots of polynomial.
  std::vector<std::complex<double> > solutions;
  PolynomialSolver solver;
  solver.init(coeffs.size() - 1);
  solver.solve(coeffs, solutions);

  // Find minimum real solution.
  double t = findMinimumRealSolution(solutions, fund_params);
  double s = triangulationObjective(t, fund_params);

  // Construct projections from the parameter t.
  cv::Mat X1;
  cv::Mat X2;
  buildParametrizedProjections(t, X1, X2, fund_params);

  // Undo canonicalization.
  X1 = A1 * X1;
  X2 = A2 * X2;

  // Return from homogeneous coordinates.
  x1 = imagePointFromHomogeneous(X1);
  x2 = imagePointFromHomogeneous(X2);

  return s;
}
