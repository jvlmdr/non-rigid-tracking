#include "vector_reader.hpp"

template<class T>
MultiviewTrackReader<T>::MultiviewTrackReader(Reader<Track<T> >& reader)
    : reader_(&reader) {}

template<class T>
MultiviewTrackReader<T>::~MultiviewTrackReader() {}

template<class T>
void MultiviewTrackReader<T>::read(const cv::FileNode& node,
                                   MultiviewTrack<T>& multiview_track) {
  std::vector<Track<T> > view_tracks;
  VectorReader<Track<T> > list_reader(*reader_);
  list_reader.read(node, view_tracks);

  int num_views = view_tracks.size();
  multiview_track.reset(num_views);

  for (int i = 0; i < num_views; i += 1) {
    multiview_track.view(i).swap(view_tracks[i]);
  }
}
