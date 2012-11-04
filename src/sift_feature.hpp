#ifndef SIFT_FEATURE_HPP_
#define SIFT_FEATURE_HPP_

#include "sift_position.hpp"
#include "descriptor.hpp"

struct SiftFeature {
  SiftPosition position;
  Descriptor descriptor;

  SiftFeature();
  SiftFeature(const SiftPosition& position, const Descriptor& descriptor);

  void swap(SiftFeature& other);
};

#endif
