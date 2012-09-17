#include "track_reader.hpp"
#include "iterator_reader.hpp"

template<class T>
MultiviewTrackReader<T>::MultiviewTrackReader(Reader<T>& reader, int num_views)
    : reader_(&reader), num_views_(num_views) {}

template<class T>
MultiviewTrackReader<T>::~MultiviewTrackReader() {}

template<class T>
bool MultiviewTrackReader<T>::read(const cv::FileNode& node,
                                   MultiviewTrack<T>& multiview_track) {
  TrackReader<T> track_reader(*reader_);

  multiview_track = MultiviewTrack<T>(num_views_);
  return readSequence(node, track_reader, multiview_track.begin());
}
