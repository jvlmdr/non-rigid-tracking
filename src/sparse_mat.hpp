#ifndef SPARSE_MAT_HPP_
#define SPARSE_MAT_HPP_

#include <opencv2/core/core.hpp>

// Computes C = A * B.
void multiply(const cv::SparseMat& A, const cv::Mat& B, cv::Mat& C);

// Adds A to B.
void addTo(const cv::SparseMat& A, cv::SparseMat& B);

// Computes C = A + B.
void add(const cv::SparseMat& A, const cv::SparseMat& B, cv::SparseMat& C);

// Computes A = diag(d) * A.
void leftMultiplyByDiag(cv::SparseMat& A, const cv::Mat& d);
// Computes A = A * diag(d).
void rightMultiplyByDiag(cv::SparseMat& A, const cv::Mat& d);

// Constructs D = diag(d)
void diag(const cv::Mat& d, cv::SparseMat& D);

#endif
