template<class T>
VectorReader<T>::VectorReader(Reader<T>& reader) : reader_(&reader) {}

template<class T>
VectorReader<T>::~VectorReader() {}

template<class T>
void VectorReader<T>::read(const cv::FileNode& parent, std::vector<T>& list) {
  const cv::FileNode& node = parent["list"];
  list.clear();

  cv::FileNodeIterator it;
  for (it = node.begin(); it != node.end(); ++it) {
    list.push_back(T());
    reader_->read(*it, list.back());
  }
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
