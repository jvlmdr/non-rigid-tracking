#ifndef DEFAULT_WRITER_HPP_
#define DEFAULT_WRITER_HPP_

#include "reader.hpp"

template<class T>
class DefaultReader : public Reader<T> {
  public:
    ~DefaultReader();
    bool read(const cv::FileNode& node, T& x);
};

template<class T>
class InlineDefaultReader : public Reader<T> {
  public:
    ~InlineDefaultReader();
    bool read(const cv::FileNode& node, T& x);
};

#include "default_reader.inl"

#endif
