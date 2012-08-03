#ifndef READER_HPP_
#define READER_HPP_

#include <opencv2/core/core.hpp>

template<class T>
class Reader {
  public:
    virtual ~Reader() {}
    virtual void read(const cv::FileNode& node, T& x) = 0;
};

// Loads anything which has an appropriate Reader.
template<class T>
bool load(const std::string& filename, T& x, Reader<T>& reader);

#include "reader.inl"

#endif
