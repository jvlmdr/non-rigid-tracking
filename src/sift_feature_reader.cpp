#include "sift_feature_reader.hpp"
#include "sift_position_reader.hpp"
#include "descriptor_reader.hpp"

SiftFeatureReader::~SiftFeatureReader() {}

bool SiftFeatureReader::read(const cv::FileNode& node, SiftFeature& feature) {
  SiftPositionReader position_reader;
  if (!position_reader.read(node, feature.position)) {
    return false;
  }

  DescriptorReader descriptor_reader;
  if (!descriptor_reader.read(node, feature.descriptor)) {
    return false;
  }

  return true;
}
