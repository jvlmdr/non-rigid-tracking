#include "sift_feature_writer.hpp"
#include "sift_position_writer.hpp"
#include "descriptor_writer.hpp"

SiftFeatureWriter::~SiftFeatureWriter() {}

void SiftFeatureWriter::write(cv::FileStorage& file,
                              const SiftFeature& feature) {
  SiftPositionWriter position_writer;
  position_writer.write(file, feature.position);

  DescriptorWriter descriptor_writer;
  descriptor_writer.write(file, feature.descriptor);
}
