template<class T>
DefaultReader<T>::~DefaultReader() {}

template<class T>
bool DefaultReader<T>::read(const cv::FileNode& node, T& x) {
  return ::read<T>(node["x"], x);
}
