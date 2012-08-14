#include "track_writer.hpp"

template<class T>
TrackListWriter<T>::TrackListWriter(Writer<Track<T> >& writer)
    : writer_(&writer) {}

template<class T>
TrackListWriter<T>::~TrackListWriter() {}

template<class T>
void TrackListWriter<T>::write(cv::FileStorage& file,
                               const TrackList<T>& tracks) {
  typename TrackList<T>::const_iterator it;

  file << "list";
  file << "[";
  for (it = tracks.begin(); it != tracks.end(); ++it) {
    file << "{";
    writer_->write(file, *it);
    file << "}";
  }
  file << "]";
}

template<class T>
bool saveTrackList(const std::string& filename,
                   const TrackList<T>& tracks,
                   Writer<T>& writer) {
  TrackWriter<T> track_writer(writer);
  TrackListWriter<T> list_writer(track_writer);
  return save(filename, tracks, list_writer);
}
