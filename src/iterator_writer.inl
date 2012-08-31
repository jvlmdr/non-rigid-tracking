template<class T, class Container>
IteratorWriter<T, Container>::IteratorWriter(Writer<T>& writer)
    : writer_(&writer) {}

template<class T, class Container>
IteratorWriter<T, Container>::~IteratorWriter() {}

template<class T, class Container>
void IteratorWriter<T, Container>::write(cv::FileStorage& file,
                                         const Container& list) {
  file << "list";
  file << "[";

  typename Container::const_iterator it;
  for (it = list.begin(); it != list.end(); ++it) {
    file << "{";
    writer_->write(file, *it);
    file << "}";
  }

  file << "]";
}
