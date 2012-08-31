template<class T, class OutputIterator>
IteratorReader<T, OutputIterator>::IteratorReader(Reader<T>& reader)
    : reader_(&reader) {}

template<class T, class OutputIterator>
IteratorReader<T, OutputIterator>::~IteratorReader() {}

template<class T, class OutputIterator>
void IteratorReader<T, OutputIterator>::read(const cv::FileNode& parent,
                                             OutputIterator& output) {
  const cv::FileNode& node = parent["list"];

  cv::FileNodeIterator it;
  for (it = node.begin(); it != node.end(); ++it) {
    T x;
    reader->read(*it, x);
    *output++ = x;
  }
}
