#ifndef CLASSIFIER_HPP_
#define CLASSIFIER_HPP_

#include <vector>
#include "descriptor.hpp"

struct Classifier {
  std::vector<double> w;
  double b;

  void swap(Classifier& other);

  template<class InputIterator>
  bool classify(InputIterator begin, InputIterator end) const;

  bool classify(const std::vector<double>& x) const;
  bool classify(const Descriptor& descriptor) const;

  template<class InputIterator>
  double score(InputIterator begin, InputIterator end) const;

  double score(const std::vector<double>& x) const;
  double score(const Descriptor& descriptor) const;
};

#include "classifier.inl"

#endif
