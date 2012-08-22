#ifndef SIMILARITY_FEATURE_HPP_
#define SIMILARITY_FEATURE_HPP_

struct SimilarityFeature {
  double x;
  double y;
  double size;
  double theta;

  SimilarityFeature();
  SimilarityFeature(double x, double y, double size, double theta);

  const double* data() const;
  double* data();
};

#endif
