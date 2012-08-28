#include "sift_feature.hpp"

SiftFeature::SiftFeature() : position(), descriptor() {}

SiftFeature::SiftFeature(const SimilarityFeature& position,
                         const Descriptor& descriptor)
    : position(position), descriptor(descriptor) {}
