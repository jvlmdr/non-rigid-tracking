#ifndef WORLD_POINT_READER_HPP_
#define WORLD_POINT_READER_HPP_

#include <opencv2/core/core.hpp>
#include "reader.hpp"

class WorldPointReader : public Reader<cv::Point3d> {
  public:
    ~WorldPointReader();
    bool read(const cv::FileNode& node, cv::Point3d& point);
};

#endif
