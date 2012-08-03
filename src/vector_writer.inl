template<class T>
VectorWriter<T>::VectorWriter(Writer<T>& writer) : writer_(&writer) {}

template<class T>
VectorWriter<T>::~VectorWriter() {}

template<class T>
void VectorWriter<T>::write(cv::FileStorage& file, const std::vector<T>& list) {
  typename std::vector<T>::const_iterator it;

  file << "[";
  for (it = list.begin(); it != list.end(); ++it) {
    writer_->write(file, *it);
  }
  file << "]";
}
