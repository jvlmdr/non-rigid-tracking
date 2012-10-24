template<class T, class OutputIterator>
bool readSequence(const cv::FileNode& node,
                  Reader<T>& reader,
                  OutputIterator output) {
  // Check node is not empty.
  if (node.type() == cv::FileNode::NONE) {
    LOG(WARNING) << "Empty file node";
    return false;
  }

  // Check node is a map.
  if (node.type() != cv::FileNode::MAP) {
    LOG(WARNING) << "Expected file node to be a map";
    return false;
  }

  const cv::FileNode& child = node["list"];

  // Check that child exists.
  if (child.type() == cv::FileNode::NONE) {
    LOG(WARNING) << "Empty file node";
    return false;
  }

  // Check that child is a sequence.
  if (child.type() != cv::FileNode::SEQ) {
    LOG(WARNING) << "Expected file node to be a sequence";
    return false;
  }

  for (cv::FileNodeIterator it = child.begin(); it != child.end(); ++it) {
    T x;
    if (!reader.read(*it, x)) {
      return false;
    }
    *output++ = x;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

template<class T, class Container>
ContainerReader<T, Container>::ContainerReader(Reader<T>& reader)
    : reader_(&reader) {}

template<class T, class Container>
ContainerReader<T, Container>::~ContainerReader() {}

template<class T, class Container>
bool ContainerReader<T, Container>::read(const cv::FileNode& node,
                                         Container& list) {
  list.clear();
  return readSequence(node, *reader_, std::back_inserter(list));
}

template<class T, class Container>
bool loadList(const std::string& file, Container& list, Reader<T>& reader) {
  ContainerReader<T, Container> list_reader(reader);
  return load(file, list, list_reader);
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
VectorReader<T>::VectorReader(Reader<T>& reader) : reader_(reader) {}

template<class T>
VectorReader<T>::~VectorReader() {}

template<class T>
bool VectorReader<T>::read(const cv::FileNode& node, std::vector<T>& list) {
  return reader_.read(node, list);
}

template<class T>
DequeReader<T>::DequeReader(Reader<T>& reader) : reader_(reader) {}

template<class T>
DequeReader<T>::~DequeReader() {}

template<class T>
bool DequeReader<T>::read(const cv::FileNode& node, std::deque<T>& list) {
  return reader_.read(node, list);
}
