template<class T>
DefaultReader<T>::~DefaultReader() {}

template<class T>
void DefaultReader<T>::read(const cv::FileNode& node, T& x) {
  x = static_cast<T>(node["x"]);
}
