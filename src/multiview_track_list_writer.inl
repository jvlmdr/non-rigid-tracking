#include "vector_writer.hpp"
#include "track_writer.hpp"
#include "multiview_track_writer.hpp"

template<class T>
MultiviewTrackListWriter<T>::MultiviewTrackListWriter(
    Writer<MultiviewTrack<T> >& writer) : writer_(&writer) {}

template<class T>
MultiviewTrackListWriter<T>::~MultiviewTrackListWriter() {}

template<class T>
void MultiviewTrackListWriter<T>::write(cv::FileStorage& file,
                                        const MultiviewTrackList<T>& tracks) {
  file << "num_views" << tracks.numViews();
  file << "tracks" << "{";
  VectorWriter<MultiviewTrack<T> > list_writer(*writer_);
  list_writer.write(file, tracks.tracks());
  file << "}";
}

template<class T>
bool saveMultiviewTrackList(const std::string& filename,
                            const MultiviewTrackList<T>& tracks,
                            Writer<T>& writer) {
  TrackWriter<T> track_writer(writer);
  MultiviewTrackWriter<T> multiview_writer(track_writer);
  MultiviewTrackListWriter<T> list_writer(multiview_writer);
  return save(filename, tracks, list_writer);
}
