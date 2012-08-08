#ifndef IMAGE_POINT_WRITER_HPP_
#define IMAGE_POINT_WRITER_HPP_

#include <opencv2/core/core.hpp>
#include "writer.hpp"

class ImagePointWriter : public Writer<cv::Point2d> {
  public:
    ~ImagePointWriter();
    void write(cv::FileStorage& file, const cv::Point2d& point);
};

#endif
