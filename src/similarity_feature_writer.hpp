#ifndef SIMILARITY_FEATURE_WRITER_HPP_
#define SIMILARITY_FEATURE_WRITER_HPP_

#include "similarity_feature.hpp"
#include "writer.hpp"

class SimilarityFeatureWriter : public Writer<SimilarityFeature> {
  public:
    ~SimilarityFeatureWriter();
    void write(cv::FileStorage& file, const SimilarityFeature& feature);
};

#endif
