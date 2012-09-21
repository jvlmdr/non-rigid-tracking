#include "sparse_mat.hpp"
#include <glog/logging.h>

void multiply(const cv::SparseMat& A, const cv::Mat& B, cv::Mat& C) {
  CHECK(A.dims() == 2);
  CHECK(A.type() == cv::DataType<double>::type);
  CHECK(B.type() == cv::DataType<double>::type);
  CHECK(A.size(1) == B.rows);

  int m = A.size(0);
  int q = B.cols;

  // Initialize result to zero.
  C = cv::Mat_<double>::zeros(m, q);

  // c_{ik} = \sum_{j=1}^{n} a_{ij} b{jk} for all i, k

  cv::SparseMatConstIterator a;
  for (a = A.begin(); a != A.end(); ++a) {
    int i = a.node()->idx[0];
    int j = a.node()->idx[1];

    // Compute contribution for each element in row of B.
    for (int k = 0; k < q; k += 1) {
      C.at<double>(i, k) += a.value<double>() * B.at<double>(j, k);
    }
  }
}

void addTo(const cv::SparseMat& A, cv::SparseMat& B) {
  // Check dimensions are the same.
  CHECK(A.dims() == B.dims());
  for (int i = 0; i < A.dims(); i += 1) {
    CHECK(A.size(i) == B.size(i));
  }

  // Check matrices are double-precision floats.
  CHECK(A.type() == cv::DataType<double>::type);
  CHECK(B.type() == cv::DataType<double>::type);

  cv::SparseMatConstIterator a;
  for (a = A.begin(); a != A.end(); ++a) {
    B.ref<double>(a.node()->idx) += a.value<double>();
  }
}

void add(const cv::SparseMat& A, const cv::SparseMat& B, cv::SparseMat& C) {
  // Check dimensions are the same.
  CHECK(A.dims() == B.dims());
  for (int i = 0; i < A.dims(); i += 1) {
    CHECK(A.size(i) == B.size(i));
  }

  // Check matrices are double-precision floats.
  CHECK(A.type() == cv::DataType<double>::type);
  CHECK(B.type() == cv::DataType<double>::type);

  C.create(A.dims(), A.size(), cv::DataType<double>::type);
  addTo(A, C);
  addTo(B, C);
}

void multiplyByDiagAlongDim(cv::SparseMat& A, const cv::Mat& d, int i) {
  CHECK(i < A.dims()) << "Matrix does not have enough dimensions";
  CHECK(int(d.total()) == A.size(i)) << "Incorrect number of diagonal elements";
  CHECK(A.type() == cv::DataType<double>::type);
  CHECK(d.type() == cv::DataType<double>::type);

  cv::SparseMatIterator a;
  for (a = A.begin(); a != A.end(); ++a) {
    int j = a.node()->idx[i];
    a.value<double>() *= d.at<double>(j);
  }
}

void leftMultiplyByDiag(cv::SparseMat& A, const cv::Mat& d) {
  multiplyByDiagAlongDim(A, d, 0);
}

void rightMultiplyByDiag(cv::SparseMat& A, const cv::Mat& d) {
  multiplyByDiagAlongDim(A, d, 1);
}

void diag(const cv::Mat& d, cv::SparseMat& D) {
  CHECK(d.type() == cv::DataType<double>::type);

  int n = d.total();

  const int ndim = 2;
  int dim[ndim] = { n, n };
  D.create(ndim, dim, cv::DataType<double>::type);

  for (int i = 0; i < n; i += 1) {
    D.ref<double>(i, i) = d.at<double>(i);
  }
}
