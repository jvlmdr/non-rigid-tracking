#ifndef CAMERA_POSE_READER_HPP_
#define CAMERA_POSE_READER_HPP_

#include "camera_pose.hpp"
#include "reader.hpp"

class CameraPoseReader : public Reader<CameraPose> {
  public:
    ~CameraPoseReader();
    void read(const cv::FileNode& node, CameraPose& pose);
};

#endif
