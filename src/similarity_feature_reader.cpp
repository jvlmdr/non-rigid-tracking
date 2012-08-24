#include "similarity_feature_reader.hpp"

SimilarityFeatureReader::~SimilarityFeatureReader() {}

void SimilarityFeatureReader::read(const cv::FileNode& node,
                                   SimilarityFeature& feature) {
  feature.x = static_cast<double>(node["x"]);
  feature.y = static_cast<double>(node["y"]);
  feature.size = static_cast<double>(node["size"]);
  feature.theta = static_cast<double>(node["angle"]);
}
