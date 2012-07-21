#include "translation_feature.hpp"

const double* TranslationFeature::data() const {
  return &x;
}

double* TranslationFeature::data() {
  return &x;
}
