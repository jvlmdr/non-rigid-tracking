template<class T>
ImagePointReader<T>::~ImagePointReader() {}

template<class T>
bool ImagePointReader<T>::read(const cv::FileNode& node, cv::Point_<T>& point) {
  if (!::read<T>(node["x"], point.x)) {
    return false;
  }

  if (!::read<T>(node["y"], point.y)) {
    return false;
  }

  return true;
}
