#include "vector_writer.hpp"
#include "track_writer.hpp"

template<class T>
MultiviewTrackListWriter<T>::MultiviewTrackListWriter(
    Writer<MultiviewTrackWriter<T> >& writer) : writer_(&writer) {}

template<class T>
MultiviewTrackListWriter<T>::~MultiviewTrackListWriter() {}

template<class T>
void MultiviewTrackListWriter<T>::write(cv::FileStorage& file,
                                        const MultiviewTrackList<T>& tracks) {
  VectorWriter<MultiviewTrack<T> > list_writer(*writer_);
  list_writer.write(tracks.tracks());
}

template<class T>
bool saveMultiviewTrackList(const std::string& filename,
                            const MultiviewTrackList<T>& tracks,
                            Writer<T>& writer) {
  TrackWriter<T> track_writer(writer);
  MultiviewTrackWriter<T> multiview_track_writer(track_writer);
  return saveList(filename, tracks.tracks(), multiview_track_writer);
}
