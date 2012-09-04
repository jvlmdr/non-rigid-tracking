#ifndef ITERATOR_READER_HPP_
#define ITERATOR_READER_HPP_

#include "reader.hpp"

template<class T, class OutputIterator>
bool readSequence(const cv::FileNode& node,
                  Reader<T>& reader,
                  OutputIterator output);

#include "iterator_reader.inl"

#endif
