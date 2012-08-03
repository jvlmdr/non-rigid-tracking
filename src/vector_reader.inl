template<class T>
VectorReader<T>::VectorReader(Reader<T>& reader) : reader_(&reader) {}

template<class T>
VectorReader<T>::~VectorReader() {}

template<class T>
void VectorReader<T>::read(const cv::FileNode& node, std::vector<T>& list) {
  cv::FileNodeIterator it;
  for (it = node.begin(); it != node.end(); ++it) {
    list.push_back(T());
    reader_->read(*it, list.back());
  }
}
