#include "sift_feature_reader.hpp"
#include "sift_position_reader.hpp"
#include "descriptor_reader.hpp"

SiftFeatureReader::~SiftFeatureReader() {}

void SiftFeatureReader::read(const cv::FileNode& node, SiftFeature& feature) {
  SiftPositionReader position_reader;
  position_reader.read(node, feature.position);

  DescriptorReader descriptor_reader;
  descriptor_reader.read(node, feature.descriptor);
}
