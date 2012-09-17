template<class T>
DefaultReader<T>::~DefaultReader() {}

template<class T>
bool DefaultReader<T>::read(const cv::FileNode& node, T& x) {
  return ::read<T>(node["x"], x);
}

template<class T>
InlineDefaultReader<T>::~InlineDefaultReader() {}

template<class T>
bool InlineDefaultReader<T>::read(const cv::FileNode& node, T& x) {
  return ::read<T>(node, x);
}
