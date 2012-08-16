#ifndef DEFAULT_WRITER_HPP_
#define DEFAULT_WRITER_HPP_

#include "writer.hpp"

template<class T>
class DefaultWriter : public Writer<T> {
  public:
    ~DefaultWriter();
    void write(cv::FileStorage& file, const T& x);
};

#include "default_writer.inl"

#endif
