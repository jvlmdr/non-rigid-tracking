namespace {

template<class T>
class PointReader : public Reader<std::pair<int, T> > {
  public:
    PointReader(Reader<T>& reader);
    ~PointReader();
    bool read(const cv::FileNode& node, std::pair<int, T>& point);

  private:
    Reader<T>* reader_;
};

template<class T>
PointReader<T>::PointReader(Reader<T>& reader) : reader_(&reader) {}

template<class T>
PointReader<T>::~PointReader() {}

template<class T>
bool PointReader<T>::read(const cv::FileNode& node, std::pair<int, T>& pair) {
  if (!::read<int>(node["t"], pair.first)) {
    return false;
  }

  if (!reader_->read(node["point"], pair.second)) {
    return false;
  }

  return true;
}

}

template<class T>
TrackReader<T>::TrackReader(Reader<T>& reader) : reader_(&reader) {}

template<class T>
TrackReader<T>::~TrackReader() {}

template<class T>
bool TrackReader<T>::read(const cv::FileNode& node, Track<T>& track) {
  const cv::FileNode& child = node["list"];
  track.clear();

  PointReader<T> point_reader(*reader_);

  for (cv::FileNodeIterator it = child.begin(); it != child.end(); ++it) {
    std::pair<int, T> pair;

    if (!point_reader.read(*it, pair)) {
      return false;
    }

    track[pair.first] = pair.second;
  }

  return true;
}
