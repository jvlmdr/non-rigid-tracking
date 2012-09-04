#include "iterator_reader.hpp"

template<class T>
VectorReader<T>::VectorReader(Reader<T>& reader) : reader_(&reader) {}

template<class T>
VectorReader<T>::~VectorReader() {}

template<class T>
bool VectorReader<T>::read(const cv::FileNode& node, std::vector<T>& list) {
  list.clear();
  return readSequence(node, *reader_, std::back_inserter(list));
}

// Loads a list of anything which has an appropriate Reader.
template<class T>
bool loadList(const std::string& filename,
              std::vector<T>& list,
              Reader<T>& reader) {
  // Create a reader for std::vector<T>.
  VectorReader<T> list_reader(reader);

  return load(filename, list, list_reader);
}
