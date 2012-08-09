#ifndef CAMERA_POSE_WRITER_HPP_
#define CAMERA_POSE_WRITER_HPP_

#include "camera_pose.hpp"
#include "writer.hpp"

class CameraPoseWriter : public Writer<CameraPose> {
  public:
    ~CameraPoseWriter();
    void write(cv::FileStorage& file, const CameraPose& pose);
};

#endif
