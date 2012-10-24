template<class T, class InputIterator>
void writeSequence(cv::FileStorage& file,
                   Writer<T>& writer,
                   InputIterator begin,
                   InputIterator end) {
  file << "list";
  file << "[";

  for (InputIterator it = begin; it != end; ++it) {
    file << "{";
    writer.write(file, *it);
    file << "}";
  }

  file << "]";
}

////////////////////////////////////////////////////////////////////////////////

template<class T, class Container>
ContainerWriter<T, Container>::ContainerWriter(Writer<T>& writer)
    : writer_(&writer) {}

template<class T, class Container>
ContainerWriter<T, Container>::~ContainerWriter() {}

template<class T, class Container>
void ContainerWriter<T, Container>::write(cv::FileStorage& file,
                                          const Container& list) {
  writeSequence(file, *writer_, list.begin(), list.end());
}

template<class T, class Container>
bool saveList(const std::string& file, Container& list, Writer<T>& writer) {
  ContainerWriter<T, Container> list_writer(writer);
  return save(file, list, list_writer);
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
VectorWriter<T>::VectorWriter(Writer<T>& writer) : writer_(writer) {}

template<class T>
VectorWriter<T>::~VectorWriter() {}

template<class T>
void VectorWriter<T>::write(cv::FileStorage& file, const std::vector<T>& list) {
  writer_.write(file, list);
}

template<class T>
DequeWriter<T>::DequeWriter(Writer<T>& writer) : writer_(writer) {}

template<class T>
DequeWriter<T>::~DequeWriter() {}

template<class T>
void DequeWriter<T>::write(cv::FileStorage& file, const std::deque<T>& list) {
  return writer_.write(file, list);
}
