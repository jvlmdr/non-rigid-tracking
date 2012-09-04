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
