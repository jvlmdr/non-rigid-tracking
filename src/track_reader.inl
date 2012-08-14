template<class T>
TrackReader<T>::TrackReader(Reader<T>& reader) : reader_(&reader) {}

template<class T>
TrackReader<T>::~TrackReader() {}

template<class T>
void TrackReader<T>::read(const cv::FileNode& parent, Track<T>& track) {
  const cv::FileNode& node = parent["list"];
  track.clear();

  cv::FileNodeIterator it;
  for (it = node.begin(); it != node.end(); ++it) {
    int t = static_cast<int>((*it)["t"]);
    T point;
    reader_->read((*it)["point"], point);

    track[t] = point;
  }
}
