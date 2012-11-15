#ifndef SCALE_SPACE_POSITION_WRITER_HPP_
#define SCALE_SPACE_POSITION_WRITER_HPP_

#include "writer.hpp"
#include "scale_space_position.hpp"

class ScaleSpacePositionWriter : public Writer<ScaleSpacePosition> {
  public:
    ~ScaleSpacePositionWriter();
    void write(cv::FileStorage& file, const ScaleSpacePosition& position);
};

#endif
