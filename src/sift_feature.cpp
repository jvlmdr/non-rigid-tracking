#include "sift_feature.hpp"

SiftFeature::SiftFeature() : position(), descriptor() {}

SiftFeature::SiftFeature(const SiftPosition& position,
                         const Descriptor& descriptor)
    : position(position), descriptor(descriptor) {}

void SiftFeature::swap(SiftFeature& other) {
  std::swap(position, other.position);
  descriptor.swap(other.descriptor);
}
