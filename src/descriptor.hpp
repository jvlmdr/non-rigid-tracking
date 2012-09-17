#ifndef DESCRIPTOR_HPP_
#define DESCRIPTOR_HPP_

#include <vector>

struct Descriptor {
  typedef std::vector<double> Data;
  Data data;

  Descriptor();
  explicit Descriptor(int n);
  Descriptor(int n, double x);

  void swap(Descriptor& other);
  void clear();
};

#endif
