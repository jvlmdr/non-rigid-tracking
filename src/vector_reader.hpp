#ifndef VECTOR_READER_HPP_
#define VECTOR_READER_HPP_

#include <vector>
#include "reader.hpp"

template<class T>
class VectorReader : public Reader<std::vector<T> > {
  public:
    VectorReader(Reader<T>& reader);
    ~VectorReader();

    bool read(const cv::FileNode& node, std::vector<T>& list);

  private:
    Reader<T>* reader_;
};

// Loads a list of anything which has an appropriate Reader.
template<class T>
bool loadList(const std::string& filename,
              std::vector<T>& list,
              Reader<T>& reader);

#include "vector_reader.inl"

#endif
