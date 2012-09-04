#include "track_reader.hpp"
#include "iterator_reader.hpp"

template<class T>
TrackListReader<T>::TrackListReader(Reader<T>& reader)
    : reader_(&reader) {}

template<class T>
TrackListReader<T>::~TrackListReader() {}

template<class T>
bool TrackListReader<T>::read(const cv::FileNode& node, TrackList<T>& tracks) {
  tracks = TrackList<T>();
  TrackReader<T> track_reader(*reader_);
  return readSequence(node, track_reader, std::back_inserter(tracks));
}

template<class T>
bool loadTrackList(const std::string& filename,
                   TrackList<T>& tracks,
                   Reader<T>& reader) {
  TrackListReader<T> list_reader(reader);
  return load(filename, tracks, list_reader);
}
