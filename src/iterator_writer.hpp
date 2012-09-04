#ifndef ITERATOR_WRITER_HPP_
#define ITERATOR_WRITER_HPP_

#include "writer.hpp"

template<class T, class InputIterator>
void writeSequence(cv::FileStorage& file,
                   Writer<T>& writer,
                   InputIterator begin,
                   InputIterator end);

#include "iterator_writer.inl"

#endif
