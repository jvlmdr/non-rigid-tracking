#ifndef MATRIX_READER_HPP_
#define MATRIX_READER_HPP_

#include <opencv2/core/core.hpp>
#include "reader.hpp"

class MatrixReader : public Reader<cv::Mat> {
  public:
    ~MatrixReader();
    bool read(const cv::FileNode& node, cv::Mat& A);
};

#endif
