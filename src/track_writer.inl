template<class T>
TrackWriter<T>::TrackWriter(Writer<T>& writer) : writer_(&writer) {}

template<class T>
TrackWriter<T>::~TrackWriter() {}

template<class T>
void TrackWriter<T>::write(cv::FileStorage& file, const Track_<T>& track) {
  file << "list";
  file << "[";

  typename Track_<T>::const_iterator point;
  for (point = track.begin(); point != track.end(); ++point) {
    file << "{";
    file << "t" << point->first;
    file << "point" << "{";
    writer_->write(file, point->second);
    file << "}";
    file << "}";
  }

  file << "]";
}
