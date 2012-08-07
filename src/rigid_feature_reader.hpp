#ifndef RIGID_FEATURE_READER_HPP_
#define RIGID_FEATURE_READER_HPP_

#include "rigid_feature.hpp"
#include "reader.hpp"

class RigidFeatureReader : public Reader<RigidFeature> {
  public:
    ~RigidFeatureReader();
    void read(const cv::FileNode& node, RigidFeature& feature);
};

#endif
