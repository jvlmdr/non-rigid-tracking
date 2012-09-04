#ifndef READER_HPP_
#define READER_HPP_

#include <opencv2/core/core.hpp>

// Describes a way of deserializing T.
template<class T>
class Reader {
  public:
    virtual ~Reader() {}
    virtual bool read(const cv::FileNode& node, T& x) = 0;
};

// Parses a variable of type T and assigns it to a variable of type X.
// Returns false if the node is empty.
template<class T, class X>
bool read(const cv::FileNode& node, X& x);

// Loads anything which has an appropriate Reader.
template<class T>
bool load(const std::string& filename, T& x, Reader<T>& reader);

#include "reader.inl"

#endif
