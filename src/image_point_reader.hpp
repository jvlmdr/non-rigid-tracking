#ifndef IMAGE_POINT_READER_HPP_
#define IMAGE_POINT_READER_HPP_

#include <opencv2/core/core.hpp>
#include "reader.hpp"

class ImagePointReader : public Reader<cv::Point2d> {
  public:
    ~ImagePointReader();
    void read(const cv::FileNode& node, cv::Point2d& point);
};

#endif
