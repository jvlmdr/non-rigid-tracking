#ifndef ITERATOR_WRITER_HPP_
#define ITERATOR_WRITER_HPP_

#include "writer.hpp"

template<class T, class Container>
class IteratorWriter : public Writer<Container> {
  public:
    IteratorWriter(Writer<T>& writer);
    ~IteratorWriter();
    void write(cv::FileStorage& file, const Container& container);

  private:
    Writer<T>* writer_;
};

#include "iterator_writer.inl"

#endif
