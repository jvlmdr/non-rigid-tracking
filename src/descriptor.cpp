#include "descriptor.hpp"

Descriptor::Descriptor() : data() {}

Descriptor::Descriptor(int n) : data(n) {}

Descriptor::Descriptor(int n, double x) : data(n, x) {}

void Descriptor::swap(Descriptor& other) {
  data.swap(other.data);
}

void Descriptor::clear() {
  data.clear();
}
