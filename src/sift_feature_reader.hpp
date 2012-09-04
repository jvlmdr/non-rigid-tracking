#ifndef SIFT_FEATURE_READER_HPP_
#define SIFT_FEATURE_READER_HPP_

#include "sift_feature.hpp"
#include "reader.hpp"

class SiftFeatureReader : public Reader<SiftFeature> {
  public:
    ~SiftFeatureReader();
    bool read(const cv::FileNode& node, SiftFeature& feature);
};

#endif
