#include "vector_reader.hpp"
#include "multiview_track_reader.hpp"
#include "track_reader.hpp"

template<class T>
MultiviewTrackListReader<T>::MultiviewTrackListReader(
    Reader<MultiviewTrack<T> >& reader) : reader_(&reader) {}

template<class T>
MultiviewTrackListReader<T>::~MultiviewTrackListReader() {}

template<class T>
void MultiviewTrackListReader<T>::read(const cv::FileNode& node,
                                       MultiviewTrackList<T>& tracks) {
  int num_views = static_cast<int>(node["num_views"]);
  tracks.reset(num_views);

  // Read tracks into vector.
  std::vector<MultiviewTrack<T> > track_list;
  VectorReader<MultiviewTrack<T> > list_reader(*reader_);
  list_reader.read(node["tracks"], track_list);

  // Swap each multiview track into data structure.
  typename std::vector<MultiviewTrack<T> >::iterator track;
  for (track = track_list.begin(); track != track_list.end(); ++track) {
    tracks.add(*track);
  }
}

template<class T>
bool loadMultiviewTrackList(const std::string& filename,
                            MultiviewTrackList<T>& tracks,
                            Reader<T>& reader) {
  TrackReader<T> track_reader(reader);
  MultiviewTrackReader<T> multiview_reader(track_reader);
  MultiviewTrackListReader<T> list_reader(multiview_reader);
  return load(filename, tracks, list_reader);
}
