template<class T>
DefaultWriter<T>::~DefaultWriter() {}

template<class T>
void DefaultWriter<T>::write(cv::FileStorage& file, const T& x) {
  file << "x" << x;
}
