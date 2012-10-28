#ifndef CAMERA_PROPERTIES_READER_HPP_
#define CAMERA_PROPERTIES_READER_HPP_

#include "camera_properties.hpp"
#include "reader.hpp"

class CameraPropertiesReader : public Reader<CameraProperties> {
  public:
    ~CameraPropertiesReader();
    bool read(const cv::FileNode& node, CameraProperties& camera);
};

#endif
