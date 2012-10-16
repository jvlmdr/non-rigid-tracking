#include <glog/logging.h>
#include <numeric>

template<class T>
MultiviewTrack<T>::MultiviewTrack() : view_tracks_(), num_frames_(0) {}

template<class T>
MultiviewTrack<T>::MultiviewTrack(int num_views)
    : view_tracks_(num_views, Track<T>()), num_frames_(0) {}

template<class T>
void MultiviewTrack<T>::swap(MultiviewTrack<T>& other) {
  view_tracks_.swap(other.view_tracks_);
  std::swap(num_frames_, other.num_frames_);
}

template<class T>
void swap(MultiviewTrack<T>& lhs, MultiviewTrack<T>& rhs) {
  lhs.swap(rhs);
}

template<class T>
const T* MultiviewTrack<T>::point(const Frame& frame) const {
  typename Track<T>::const_iterator result;

  result = view_tracks_[frame.view].find(frame.time);

  if (result == view_tracks_[frame.view].end()) {
    // Frame was not found in track index.
    return NULL;
  } else {
    // Frame was found.
    return &result->second;
  }
}

template<class T>
T* MultiviewTrack<T>::point(const Frame& frame) {
  typename Track<T>::iterator result;

  result = view_tracks_[frame.view].find(frame.time);

  if (result == view_tracks_[frame.view].end()) {
    // Frame was not found in track index.
    return NULL;
  } else {
    // Frame was found.
    return &result->second;
  }
}

template<class T>
const Track<T>& MultiviewTrack<T>::view(int i) const {
  return view_tracks_[i];
}

template<class T>
Track<T>& MultiviewTrack<T>::view(int i) {
  return view_tracks_[i];
}

template<class T>
typename MultiviewTrack<T>::iterator
MultiviewTrack<T>::begin() {
  return view_tracks_.begin();
}

template<class T>
typename MultiviewTrack<T>::iterator
MultiviewTrack<T>::end() {
  return view_tracks_.end();
}

template<class T>
typename MultiviewTrack<T>::const_iterator
MultiviewTrack<T>::begin() const {
  return view_tracks_.begin();
}

template<class T>
typename MultiviewTrack<T>::const_iterator
MultiviewTrack<T>::end() const {
  return view_tracks_.end();
}

template<class T>
int MultiviewTrack<T>::numViews() const {
  return view_tracks_.size();
}

template<class T>
int MultiviewTrack<T>::numFrames() const {
  return num_frames_;
}

template<class T>
bool MultiviewTrack<T>::empty() const {
  int num_views = view_tracks_.size();

  for (int i = 0; i < num_views; i += 1) {
    // If any view's track is non-empty, then the multi-view track is non-empty.
    if (!view_tracks_[i].empty()) {
      return false;
    }
  }

  return true;
}

template<class T>
int MultiviewTrack<T>::numImageFeatures() const {
  return std::accumulate(view_tracks_.begin(), view_tracks_.end(), 0,
      addTrackSize<T>);
}

template<class T>
int MultiviewTrack<T>::numViewsPresent() const {
  int num_views = view_tracks_.size();
  int count = 0;

  for (int i = 0; i < num_views; i += 1) {
    if (!view_tracks_[i].empty()) {
      count += 1;
    }
  }

  return count;
}

template<class T>
int addMultiviewTrackImageFeatures(int x, const MultiviewTrack<T>& track) {
  return x + track.numImageFeatures();
}
