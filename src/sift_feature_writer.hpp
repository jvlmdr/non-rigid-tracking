#ifndef SIFT_FEATURE_WRITER_HPP_
#define SIFT_FEATURE_WRITER_HPP_

#include "writer.hpp"
#include "sift_feature.hpp"

class SiftFeatureWriter : public Writer<SiftFeature> {
  public:
    ~SiftFeatureWriter();
    void write(cv::FileStorage& file, const SiftFeature& feature);
};

#endif
