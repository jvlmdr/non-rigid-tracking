template<class T>
ImagePointWriter<T>::~ImagePointWriter() {}

template<class T>
void ImagePointWriter<T>::write(cv::FileStorage& file,
                                const cv::Point_<T>& point) {
  file << "x" << point.x << "y" << point.y;
}
