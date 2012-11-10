#ifndef IMAGE_POINT_WRITER_HPP_
#define IMAGE_POINT_WRITER_HPP_

#include <opencv2/core/core.hpp>
#include "writer.hpp"

template<class T>
class ImagePointWriter : public Writer<cv::Point_<T> > {
  public:
    ~ImagePointWriter();
    void write(cv::FileStorage& file, const cv::Point_<T>& point);
};

#include "image_point_writer.inl"

#endif
