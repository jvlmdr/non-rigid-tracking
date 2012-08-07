#ifndef ROOTS_HPP_
#define ROOTS_HPP_

#include <gsl/gsl_poly.h>
#include <vector>
#include <complex>

class PolynomialSolver {
  public:
    PolynomialSolver();
    ~PolynomialSolver();

    void init(int n);

    void solve(const std::vector<double>& a,
               std::vector<std::complex<double> >& x);

  private:
    int n_;
    gsl_poly_complex_workspace* workspace_;
};

#endif
