#ifndef SIFT_POSITION_READER_HPP_
#define SIFT_POSITION_READER_HPP_

#include "sift_position.hpp"
#include "reader.hpp"

class SiftPositionReader : public Reader<SiftPosition> {
  public:
    ~SiftPositionReader();
    bool read(const cv::FileNode& node, SiftPosition& feature);
};

#endif
