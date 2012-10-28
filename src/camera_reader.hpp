#ifndef CAMERA_READER_HPP_
#define CAMERA_READER_HPP_

#include "camera.hpp"
#include "reader.hpp"

class CameraReader : public Reader<Camera> {
  public:
    ~CameraReader();
    bool read(const cv::FileNode& node, Camera& camera);
};

#endif
