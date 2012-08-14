#include "track_reader.hpp"

template<class T>
TrackListReader<T>::TrackListReader(Reader<Track_<T> >& reader)
    : reader_(&reader) {}

template<class T>
TrackListReader<T>::~TrackListReader() {}

template<class T>
void TrackListReader<T>::read(const cv::FileNode& parent,
                              TrackList_<T>& tracks) {
  // TODO: Avoid duplicating code from VectorReader.
  const cv::FileNode& node = parent["list"];
  tracks.clear();

  cv::FileNodeIterator it;
  for (it = node.begin(); it != node.end(); ++it) {
    tracks.push_back(Track_<T>());
    reader_->read(*it, tracks.back());
  }
}

template<class T>
bool loadTrackList(const std::string& filename,
                   TrackList_<T>& tracks,
                   Reader<T>& reader) {
  TrackReader<T> track_reader(reader);
  TrackListReader<T> list_reader(track_reader);
  return load(filename, tracks, list_reader);
}
