#ifndef SIFT_FEATURE_HPP_
#define SIFT_FEATURE_HPP_

#include "similarity_feature.hpp"
#include "descriptor.hpp"

struct SiftFeature {
  SimilarityFeature position;
  Descriptor descriptor;

  SiftFeature();
  SiftFeature(const SimilarityFeature& position,
              const Descriptor& descriptor);
};

#endif
