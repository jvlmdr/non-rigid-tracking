#include "rigid_feature_writer.hpp"

RigidFeatureWriter::~RigidFeatureWriter() {}

void RigidFeatureWriter::write(cv::FileStorage& file,
                               const RigidFeature& feature) {
  file << "x" << feature.x;
  file << "y" << feature.y;
  file << "size" << feature.size;
  file << "angle" << feature.theta;
}
