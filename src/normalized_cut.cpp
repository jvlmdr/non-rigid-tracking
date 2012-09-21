#include "normalized_cut.hpp"
#include <opencv2/core/core.hpp>
#include <glog/logging.h>
#include "sparse_mat.hpp"
#include "arssym.h"

namespace {

class OpencvArpackMatrix {
  public:
    OpencvArpackMatrix(const cv::SparseMat& mat) : mat_(&mat) {}

    int nrows() const {
      return mat_->size(0);
    }

    int ncols() const {
      return mat_->size(1);
    }

    void MultMv(double* v, double* w) {
      cv::Mat x(ncols(), 1, cv::DataType<double>::type, v, false);
      cv::Mat y(nrows(), 1, cv::DataType<double>::type, w, false);

      multiply(*mat_, x, y);
    }

  private:
    const cv::SparseMat* mat_;
};

std::string eigenvalueList(
    ARSymStdEig<double, OpencvArpackMatrix>& problem,
    int nconv) {
  std::ostringstream ss;

  for (int i = 0; i < nconv; i += 1) {
    if (i != 0) {
      ss << ", ";
    }
    ss << problem.Eigenvalue(i);
  }

  return ss.str();
}

}

bool smallestEigenvector(const cv::SparseMat& A,
                         int k,
                         std::vector<double>* x,
                         double* lambda,
                         int max_iter) {
  int n = A.size(0);
  OpencvArpackMatrix matrix(A);

  ARSymStdEig<double, OpencvArpackMatrix> problem(matrix.ncols(), k + 1,
      &matrix, &OpencvArpackMatrix::MultMv, "SM");
  problem.ChangeMaxit(max_iter);

  int nconv = problem.FindEigenvectors();
  DLOG(INFO) << "lambda = [" << eigenvalueList(problem, nconv) << "]";

  if (nconv < k + 1) {
    return false;
  }

  if (x != NULL) {
    x->clear();
    for (int i = 0; i < n; i += 1) {
      x->push_back(problem.Eigenvector(k, i));
    }
  }

  if (lambda != NULL) {
    *lambda = problem.Eigenvalue(k);
  }

  return true;
}
