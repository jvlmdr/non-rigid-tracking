#ifndef CAMERA_WRITER_HPP_
#define CAMERA_WRITER_HPP_

#include "camera.hpp"
#include "writer.hpp"

class CameraWriter : public Writer<Camera> {
  public:
    ~CameraWriter();
    void write(cv::FileStorage& file, const Camera& camera);
};

#endif
