#include "sparse_mat.hpp"
#include "gtest/gtest.h"

TEST(SparseTimesDense, Basic) {
  // Construct 3x5 matrix A.
  const int ndims = 2;
  int dims[ndims] = { 3, 5 };
  cv::SparseMat A(ndims, dims, cv::DataType<double>::type);
  A.ref<double>(0, 3) = -3;
  A.ref<double>(1, 0) = 1;
  A.ref<double>(1, 1) = 2;
  A.ref<double>(2, 4) = -1;

  // Construct 5x4 matrix B.
  cv::Mat B = (cv::Mat_<double>(5, 4) <<
       1,  0, -1,  0,
       1,  1,  0,  0,
       1,  0,  0,  1,
       1,  1,  0,  0,
       1,  0,  2,  0);

  // Multiply A by B.
  cv::Mat C;
  multiply(A, B, C);

  // Do dense multiplication for reference.
  cv::Mat dense_A;
  A.copyTo(dense_A);
  cv::Mat ref_C;
  ref_C = dense_A * B;

  // Compare result.
  cv::Mat E = C - ref_C;
  double e = E.dot(E);
  ASSERT_EQ(e, 0.);
}

TEST(SparsePlusSparse, Basic) {
  const int ndims = 2;
  int dims[ndims] = { 4, 5 };

  cv::SparseMat A(ndims, dims, cv::DataType<double>::type);
  A.ref<double>(0, 3) = -3;
  A.ref<double>(1, 0) = 1;
  A.ref<double>(1, 1) = 2;
  A.ref<double>(3, 4) = -1;

  cv::SparseMat B(ndims, dims, cv::DataType<double>::type);
  B.ref<double>(0, 0) = 1;
  B.ref<double>(1, 1) = -4;
  B.ref<double>(2, 1) = -3;
  B.ref<double>(3, 3) = 2;
  B.ref<double>(3, 4) = 1;

  cv::SparseMat C;
  add(A, B, C);

  // Convert to dense to make comparison.
  cv::Mat dense_C;
  C.copyTo(dense_C);

  // Do dense addition for reference.
  cv::Mat dense_A;
  cv::Mat dense_B;
  A.copyTo(dense_A);
  B.copyTo(dense_B);
  cv::Mat ref_C;
  ref_C = dense_A + dense_B;

  // Compare result.
  cv::Mat E = dense_C - ref_C;
  double e = E.dot(E);
  ASSERT_EQ(e, 0.);
}
