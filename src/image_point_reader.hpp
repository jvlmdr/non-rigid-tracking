#ifndef IMAGE_POINT_READER_HPP_
#define IMAGE_POINT_READER_HPP_

#include <opencv2/core/core.hpp>
#include "reader.hpp"

template<class T>
class ImagePointReader : public Reader<cv::Point_<T> > {
  public:
    ~ImagePointReader();
    bool read(const cv::FileNode& node, cv::Point_<T>& point);
};

#include "image_point_reader.inl"

#endif
