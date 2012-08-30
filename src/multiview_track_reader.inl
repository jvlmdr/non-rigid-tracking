#include "vector_reader.hpp"

template<class T>
MultiviewTrackReader<T>::MultiviewTrackReader(Reader<Track<T> >& reader)
    : reader_(&reader) {}

template<class T>
MultiviewTrackReader<T>::~MultiviewTrackReader() {}

template<class T>
void MultiviewTrackReader<T>::read(const cv::FileNode& node,
                                   MultiviewTrack<T>& multiview_track) {
  std::vector<Track<T> > views;
  VectorReader<Track<T> > list_reader(*reader_);
  list_reader.read(node, views);

  int num_views = views.size();
  multiview_track.reset(num_views);

  // Iterate through views.
  for (int view = 0; view < num_views; view += 1) {
    // Iterate through track in each view.
    typename Track<T>::const_iterator point;
    for (point = views[view].begin(); point != views[view].end(); ++point) {
      // Add each point to the track.
      int time = point->first;
      multiview_track.set(Frame(view, time), point->second);
    }
  }
}
