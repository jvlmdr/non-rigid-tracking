#ifndef DEFAULT_WRITER_HPP_
#define DEFAULT_WRITER_HPP_

#include "reader.hpp"

template<class T>
class DefaultReader : public Reader<T> {
  public:
    ~DefaultReader();
    void read(const cv::FileNode& node, T& x);
};

#include "default_reader.inl"

#endif
