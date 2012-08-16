#include <algorithm>

template<class T>
MultiviewTrackList<T>::MultiviewTrackList()
    : tracks_(), num_views_(0), num_frames_(0) {}

template<class T>
MultiviewTrackList<T>::MultiviewTrackList(int num_views)
    : tracks_(), num_views_(num_views), num_frames_(0) {}

template<class T>
void MultiviewTrackList<T>::reset(int num_views) {
  tracks_.clear();
  num_views_ = num_views;
  num_frames_ = 0;
}

template<class T>
void MultiviewTrackList<T>::swap(MultiviewTrackList<T>& other) {
  tracks_.swap(other.tracks_);
  std::swap(num_views_, other.num_views_);
  std::swap(num_frames_, other.num_frames_);
}

template<class T>
void MultiviewTrackList<T>::add(MultiviewTrack<T>& track) {
  // Swap into list.
  tracks_.push_back(MultiviewTrack<T>());
  tracks_.back().swap(track);

  // Update number of frames if necessary.
  num_frames_ = std::max(num_frames_, track.numFrames());
}

// Provides read access to the tracks in one view.
template<class T>
const MultiviewTrack<T>& MultiviewTrackList<T>::track(int id) const {
  return tracks_[id];
}

template<class T>
const std::vector<MultiviewTrack<T> >& MultiviewTrackList<T>::tracks() const {
  return tracks_;
}

template<class T>
int MultiviewTrackList<T>::numTracks() const {
  return tracks_.size();
}

template<class T>
int MultiviewTrackList<T>::numViews() const {
  return num_views_;
}

template<class T>
int MultiviewTrackList<T>::numFrames() const {
  return num_frames_;
}
