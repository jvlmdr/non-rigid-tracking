#ifndef TRANSLATION_FEATURE_HPP_
#define TRANSLATION_FEATURE_HPP_

struct TranslationFeature {
  double x;
  double y;

  const double* data() const;
  double* data();
};

#endif
