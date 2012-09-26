#include "matrix_reader.hpp"

bool readMatrix(const cv::FileNode& node, cv::Mat& A) {
  if (node.type() == cv::FileNode::NONE) {
    return false;
  }

  node >> A;

  return true;
}

MatrixReader::~MatrixReader() {}

bool MatrixReader::read(const cv::FileNode& node, cv::Mat& A) {
  if (node.type() != cv::FileNode::MAP) {
    return false;
  }

  return readMatrix(node["matrix"], A);
}
