#ifndef RIGID_FEATURE_WRITER_HPP_
#define RIGID_FEATURE_WRITER_HPP_

#include "rigid_feature.hpp"
#include "writer.hpp"

class RigidFeatureWriter : public Writer<RigidFeature> {
  public:
    ~RigidFeatureWriter();
    void write(cv::FileStorage& file, const RigidFeature& feature);
};

#endif
