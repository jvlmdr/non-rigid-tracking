#ifndef SIMILARITY_FEATURE_READER_HPP_
#define SIMILARITY_FEATURE_READER_HPP_

#include "similarity_feature.hpp"
#include "reader.hpp"

class SimilarityFeatureReader : public Reader<SimilarityFeature> {
  public:
    ~SimilarityFeatureReader();
    void read(const cv::FileNode& node, SimilarityFeature& feature);
};

#endif
