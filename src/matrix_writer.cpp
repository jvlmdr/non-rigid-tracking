#include "matrix_writer.hpp"

MatrixWriter::~MatrixWriter() {}

void MatrixWriter::write(cv::FileStorage& file, const cv::Mat& A) {
  file << "matrix" << A;
}
