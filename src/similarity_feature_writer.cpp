#include "similarity_feature_writer.hpp"

SimilarityFeatureWriter::~SimilarityFeatureWriter() {}

void SimilarityFeatureWriter::write(cv::FileStorage& file,
                                    const SimilarityFeature& feature) {
  file << "x" << feature.x;
  file << "y" << feature.y;
  file << "size" << feature.size;
  file << "angle" << feature.theta;
}
