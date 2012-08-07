#include "roots.hpp"

PolynomialSolver::PolynomialSolver() : n_(0), workspace_(NULL) {}

PolynomialSolver::~PolynomialSolver() {
  if (workspace_ != NULL) {
    gsl_poly_complex_workspace_free(workspace_);
  }
}

// Initializes a polynomial solver of degree n.
void PolynomialSolver::init(int n) {
  n_ = n;
  workspace_ = gsl_poly_complex_workspace_alloc(n_ + 1);
}

// Finds the roots of the polynomial a[0] + a[1] x + ... + a[n] x^n.
// http://www.mathworks.com/help/techdoc/ref/roots.html
void PolynomialSolver::solve(const std::vector<double>& a,
                             std::vector<std::complex<double> >& x) {
  // Allocate sufficient space for complex solutions and solve.
  std::vector<double> z(2 * n_);
  gsl_poly_complex_solve(&a.front(), n_ + 1, workspace_, &z.front());

  // Copy into complex data structure.
  x.clear();
  for (int i = 0; i < n_; i += 1) {
    x.push_back(std::complex<double>(z[2 * i], z[2 * i + 1]));
  }
}
