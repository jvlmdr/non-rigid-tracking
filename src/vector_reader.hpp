#ifndef VECTOR_READER_HPP_
#define VECTOR_READER_HPP_

#include <vector>
#include "reader.hpp"

template<class T>
class VectorReader : public Reader<std::vector<T> > {
  public:
    VectorReader(Reader<T>& reader);
    ~VectorReader();

    void read(const cv::FileNode& node, std::vector<T>& list);

  private:
    Reader<T>* reader_;
};

#include "vector_reader.inl"

#endif
