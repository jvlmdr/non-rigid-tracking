#include "classifier.hpp"

void Classifier::swap(Classifier& other) {
  w.swap(other.w);
  std::swap(b, other.b);
}

bool Classifier::classify(const std::vector<double>& x) const {
  return classify(x.begin(), x.end());
}

bool Classifier::classify(const Descriptor& descriptor) const {
  return classify(descriptor.data);
}

double Classifier::score(const std::vector<double>& x) const {
  return score(x.begin(), x.end());
}

double Classifier::score(const Descriptor& descriptor) const {
  return score(descriptor.data);
}
