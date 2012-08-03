#ifndef WRITER_HPP_
#define WRITER_HPP_

#include <opencv2/core/core.hpp>

// Describes a way of serializing T.
template<class T>
class Writer {
  public:
    virtual ~Writer() {}
    virtual void write(cv::FileStorage& file, const T& x) = 0;
};

// Saves anything which has an appropriate Writer.
template<class T>
bool save(const std::string& filename, const T& x, Writer<T>& writer);

#include "writer.inl"

#endif
