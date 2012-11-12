#ifndef SCALE_SPACE_POSITION_READER_
#define SCALE_SPACE_POSITION_READER_

#include "scale_space_position.hpp"
#include "reader.hpp"

class ScaleSpacePositionReader : public Reader<ScaleSpacePosition> {
  public:
    ~ScaleSpacePositionReader();
    bool read(const cv::FileNode& node, ScaleSpacePosition& position);
};

#endif
