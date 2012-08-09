#ifndef MATRIX_WRITER_HPP_
#define MATRIX_WRITER_HPP_

#include <opencv2/core/core.hpp>
#include "writer.hpp"

class MatrixWriter : public Writer<cv::Mat> {
  public:
    ~MatrixWriter();
    void write(cv::FileStorage& file, const cv::Mat& A);
};

#endif
