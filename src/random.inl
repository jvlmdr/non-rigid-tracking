#include <algorithm>

class GenerateFunction {
  public:
    GenerateFunction(boost::random::mt19937& generator);
    int operator()(int n);

  private:
    boost::random::mt19937* generator_;
};

template<class RandomAccessIterator>
void randomShuffle(RandomAccessIterator first,
                   RandomAccessIterator last,
                   boost::random::mt19937& generator) {
  GenerateFunction function(generator);
  std::random_shuffle(first, last, function);
}
