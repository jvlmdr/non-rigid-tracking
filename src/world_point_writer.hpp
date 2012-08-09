#ifndef WORLD_POINT_WRITER_HPP_
#define WORLD_POINT_WRITER_HPP_

#include <opencv2/core/core.hpp>
#include "writer.hpp"

class WorldPointWriter : public Writer<cv::Point3d> {
  public:
    ~WorldPointWriter();
    void write(cv::FileStorage& file, const cv::Point3d& point);
};

#endif
