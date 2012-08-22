#include "similarity_feature.hpp"

SimilarityFeature::SimilarityFeature() {}

SimilarityFeature::SimilarityFeature(double x,
                                     double y,
                                     double size,
                                     double theta)
      : x(x), y(y), size(size), theta(theta) {}

const double* SimilarityFeature::data() const {
  return &x;
}

double* SimilarityFeature::data() {
  return &x;
}
