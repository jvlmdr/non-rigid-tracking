#ifndef CAMERA_PROPERTIES_WRITER_HPP_
#define CAMERA_PROPERTIES_WRITER_HPP_

#include "camera_properties.hpp"
#include "writer.hpp"

class CameraPropertiesWriter : public Writer<CameraProperties> {
  public:
    ~CameraPropertiesWriter();
    void write(cv::FileStorage& file, const CameraProperties& properties);
};

#endif
