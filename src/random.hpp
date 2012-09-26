#ifndef RANDOM_HPP_
#define RANDOM_HPP_

#include <boost/random/mersenne_twister.hpp>

template<class RandomAccessIterator>
void randomShuffle(RandomAccessIterator first,
                   RandomAccessIterator last,
                   boost::random::mt19937& generator);

#include "random.inl"

#endif
