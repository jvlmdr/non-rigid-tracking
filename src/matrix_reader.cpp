#include "matrix_reader.hpp"

MatrixReader::~MatrixReader() {}

void MatrixReader::read(const cv::FileNode& node, cv::Mat& A) {
  node["matrix"] >> A;
}
