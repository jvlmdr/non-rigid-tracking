#include "random.hpp"
#include <boost/random/uniform_int_distribution.hpp>

GenerateFunction::GenerateFunction(boost::random::mt19937& generator)
    : generator_(&generator) {}

int GenerateFunction::operator()(int n) {
  boost::random::uniform_int_distribution<> dist(0, n - 1);
  return dist(*generator_);
}
