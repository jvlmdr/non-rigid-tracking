#ifndef VECTOR_WRITER_HPP_
#define VECTOR_WRITER_HPP_

#include <vector>
#include "writer.hpp"

template<class T>
class VectorWriter : public Writer<std::vector<T> > {
  public:
    VectorWriter(Writer<T>& writer);
    ~VectorWriter();

    void write(cv::FileStorage& file, const std::vector<T>& x);

  private:
    Writer<T>* writer_;
};

#include "vector_writer.inl"

#endif
