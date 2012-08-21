#include <glog/logging.h>

template<class T>
MultiviewTrack<T>::MultiviewTrack() : tracks_(), num_frames_(0) {}

template<class T>
MultiviewTrack<T>::MultiviewTrack(int num_views)
    : tracks_(num_views, Track<T>()), num_frames_(0) {}

template<class T>
void MultiviewTrack<T>::reset(int num_views) {
  num_frames_ = 0;
  tracks_.assign(num_views, Track<T>());
}

template<class T>
void MultiviewTrack<T>::swap(MultiviewTrack<T>& other) {
  tracks_.swap(other.tracks_);
  std::swap(num_frames_, other.num_frames_);
}

template<class T>
void MultiviewTrack<T>::add(const Frame& frame, const T& x) {
  tracks_[frame.view][frame.time] = x;
  if (frame.time > num_frames_ - 1) {
    num_frames_ = frame.time + 1;
  }
}

template<class T>
const Track<T>& MultiviewTrack<T>::track(int view) const {
  return tracks_[view];
}

template<class T>
const std::vector<Track<T> >& MultiviewTrack<T>::tracks() const {
  return tracks_;
}

template<class T>
int MultiviewTrack<T>::numViews() const {
  return tracks_.size();
}

template<class T>
int MultiviewTrack<T>::numFrames() const {
  return num_frames_;
}

template<class T>
bool MultiviewTrack<T>::empty() const {
  int num_views = tracks_.size();

  for (int i = 0; i < num_views; i += 1) {
    // If any view's track is non-empty, then the multi-view track is non-empty.
    if (!tracks_[i].empty()) {
      return false;
    }
  }

  return true;
}
