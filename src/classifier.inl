#include <glog/logging.h>

template<class InputIterator>
bool Classifier::classify(InputIterator begin, InputIterator end) const {
  return score(begin, end) > 0;
}

template<class InputIterator>
double Classifier::score(InputIterator begin, InputIterator end) const {
  std::vector<double>::const_iterator u = w.begin();
  InputIterator& v = begin;

  double y = b;

  while (u != w.end() && v != end) {
    y += *u * *v;

    ++u;
    ++v;
  }

  CHECK(u == w.end() && v == end) << "Did not reach end together";

  return y;
}
