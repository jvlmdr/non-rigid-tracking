#ifndef SIMILARITY_FEATURE_HPP_
#define SIMILARITY_FEATURE_HPP_

const double SIFT_SIZE_TO_SIGMA = 1.;

struct SimilarityFeature {
  double x;
  double y;
  double size;
  double theta;

  SimilarityFeature();
  SimilarityFeature(double x, double y, double size, double theta);
};

#endif
