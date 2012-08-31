#ifndef ITERATOR_READER_HPP_
#define ITERATOR_READER_HPP_

#include "reader.hpp"

template<class T, class OutputIterator>
class IteratorReader : public Reader<OutputIterator> {
  public:
    IteratorReader(Reader<T>& reader);
    ~IteratorReader();
    void read(const cv::FileNode& parent, OutputIterator& output);

  private:
    Reader<T>* reader_;
};

#include "iterator_reader.inl"

#endif
