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
const T* MultiviewTrack<T>::point(const ImageIndex& frame) const {
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
T* MultiviewTrack<T>::point(const ImageIndex& frame) {
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
int MultiviewTrack<T>::firstFrameNumber() const {
  bool valid = false;
  int t = -1;

  typename Container::const_iterator view;
  for (view = view_tracks_.begin(); view != view_tracks_.end(); ++view) {
    // Check that track for this view isn't empty.
    if (!view->empty()) {
      // Get time of first frame.
      int u = view->begin()->first;

      // Find min t.
      if (!valid || u < t) {
        t = u;
        valid = true;
      }
    }
  }

  return t;
}

template<class T>
int MultiviewTrack<T>::lastFrameNumber() const {
  bool valid = false;
  int t = -1;

  typename Container::const_iterator view;
  for (view = view_tracks_.begin(); view != view_tracks_.end(); ++view) {
    // Check that track for this view isn't empty.
    if (!view->empty()) {
      // Get time of last frame.
      int u = view->rbegin()->first;

      // Find max t.
      if (!valid || u > t) {
        t = u;
        valid = true;
      }
    }
  }

  return t;
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

////////////////////////////////////////////////////////////////////////////////

template<class T>
MultiviewTrack<T>::TimeIterator::TimeIterator()
    : cursors_(), time_(-1) {}

template<class T>
MultiviewTrack<T>::TimeIterator::TimeIterator(
    const MultiviewTrack<T>& track) : cursors_(), time_(0) {
  bool valid = false;

  // Populate cursor list for each view.
  typename MultiviewTrack<T>::const_iterator view;
  for (view = track.begin(); view != track.end(); ++view) {
    // Add cursor at the start of the track for this view.
    cursors_.push_back(TrackIterator<T>(*view));

    // Set time to earliest frame in all views.
    // Check that the track for this view is not empty.
    if (!view->empty()) {
      int t = view->begin()->first;
      if (!valid || t < time_) {
        time_ = t;
        valid = true;
      }
    }
  }
}

template<class T>
MultiviewTrack<T>::TimeIterator::TimeIterator(
    const MultiviewTrack<T>& track,
    int time)
    : cursors_(), time_(time) {
  // Populate cursor list for each view.
  typename MultiviewTrack<T>::const_iterator view;

  for (view = track.begin(); view != track.end(); ++view) {
    // Add cursor at the start of the track for this view.
    cursors_.push_back(TrackIterator<T>(*view, time_));
  }
}

template<class T>
void MultiviewTrack<T>::TimeIterator::next() {
  typename CursorList::iterator cursor;
  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // Check that cursor has not reached end.
    if (!cursor->end()) {
      // Advance cursor if it points to the current frame.
      if (cursor->time() == time_) {
        cursor->next();
      }
    }
  }

  time_ += 1;
}

template<class T>
bool MultiviewTrack<T>::TimeIterator::end() const {
  typename CursorList::const_iterator cursor;
  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // If at least one view has not ended, the multiview track has not ended.
    if (!cursor->end()) {
      return false;
    }
  }

  return true;
}

template<class T>
int MultiviewTrack<T>::TimeIterator::time() const {
  return time_;
}

template<class T>
void MultiviewTrack<T>::TimeIterator::get(std::map<int, T>& points) const {
  points.clear();

  typename CursorList::const_iterator cursor;
  int i = 0;
  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // Check that the track has not ended.
    if (!cursor->end()) {
      // If the cursor points to the current frame, add to the output.
      if (cursor->time() == time_) {
        points[i] = cursor->get();
      }
    }

    i += 1;
  }
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
MultiviewTrack<T>::FeatureIterator::FeatureIterator()
    : track_(), view_(), feature_() {}

template<class T>
MultiviewTrack<T>::FeatureIterator::FeatureIterator(
    const MultiviewTrack<T>& track)
    : track_(&track), view_(), feature_() {}

template<class T>
void MultiviewTrack<T>::FeatureIterator::begin() {
  view_ = track_->begin();

  while (view_ != track_->end()) {
    feature_ = view_->begin();

    if (feature_ != view_->end()) {
      return;
    }

    ++view_;
  }
}

template<class T>
void MultiviewTrack<T>::FeatureIterator::next() {
  // If we've reached the end, do nothing.
  if (view_ == track_->end()) {
    return;
  }

  // Advance our position in the list of features.
  ++feature_;

  // If we reached the end of the list, move to the next non-empty list.
  while (feature_ == view_->end()) {
    view_++;

    // If we reached the end, we're done.
    if (view_ == track_->end()) {
      return;
    }

    // Take first feature in view.
    feature_ = view_->begin();
  }
}

template<class T>
bool MultiviewTrack<T>::FeatureIterator::end() const {
  return (view_ == track_->end());
}

template<class T>
std::pair<ImageIndex, const T*>
MultiviewTrack<T>::FeatureIterator::get() const {
  // Get view index as iterator offset.
  int view = view_ - track_->begin();
  // Get time index of feature.
  int time = feature_->first;

  return std::make_pair(ImageIndex(view, time), &feature_->second);
}
