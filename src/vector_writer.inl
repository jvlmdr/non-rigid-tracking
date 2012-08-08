template<class T>
VectorWriter<T>::VectorWriter(Writer<T>& writer) : writer_(&writer) {}

template<class T>
VectorWriter<T>::~VectorWriter() {}

template<class T>
void VectorWriter<T>::write(cv::FileStorage& file, const std::vector<T>& list) {
  typename std::vector<T>::const_iterator it;

  file << "list";
  file << "[";
  for (it = list.begin(); it != list.end(); ++it) {
    file << "{";
    writer_->write(file, *it);
    file << "}";
  }
  file << "]";
}

// Saves a list of anything which has an appropriate Writer.
template<class T>
bool saveList(const std::string& filename,
              const std::vector<T>& list,
              Writer<T>& writer) {
  // Create a writer for std::vector<T>.
  VectorWriter<T> list_writer(writer);

  return save(filename, list, list_writer);
}
