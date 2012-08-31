#ifndef SIFT_POSITION_WRITER_HPP_
#define SIFT_POSITION_WRITER_HPP_

#include "sift_position.hpp"
#include "writer.hpp"

class SiftPositionWriter : public Writer<SiftPosition> {
  public:
    ~SiftPositionWriter();
    void write(cv::FileStorage& file, const SiftPosition& feature);
};

#endif
