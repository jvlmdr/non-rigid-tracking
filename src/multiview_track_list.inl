#include <algorithm>
#include <glog/logging.h>

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
  CHECK(num_views_ == track.numViews());

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

////////////////////////////////////////////////////////////////////////////////

template<class T>
SingleViewTimeIterator<T>::SingleViewTimeIterator()
    : view_(-1), time_(0), cursors_() {}

template<class T>
SingleViewTimeIterator<T>::SingleViewTimeIterator(
    const MultiviewTrackList<T>& tracks,
    int view)
    : view_(view), time_(0), cursors_() {
  // Populate cursor list.
  typename std::vector<MultiviewTrack<T> >::const_iterator multiview_track;

  for (multiview_track = tracks.tracks().begin();
       multiview_track != tracks.tracks().end();
       ++multiview_track) {
    // Add cursor at beginning of track for given view.
    cursors_.push_back(TrackIterator<T>(multiview_track->track(view_)));
  }
}

template<class T>
void SingleViewTimeIterator<T>::next() {
  // Iterate through features.
  typename CursorList::iterator cursor;

  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // Has this track ended already?
    if (!cursor->end()) {
      // Track has not ended.
      // Did the next observation occur at this frame?
      if (cursor->time() == time_) {
        // It did. Advance the cursor.
        cursor->next();
      }
    }
  }

  time_ += 1;
}

template<class T>
bool SingleViewTimeIterator<T>::end() const {
  // Iterate through features.
  typename CursorList::const_iterator cursor;
  
  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // Has this track ended already?
    if (!cursor->end()) {
      // Track has not ended.
      return false;
    }
  }

  return true;
}

template<class T>
int SingleViewTimeIterator<T>::time() const {
  return time_;
}

template<class T>
void SingleViewTimeIterator<T>::get(std::map<int, T>& points) const {
  points.clear();

  // Iterate through features.
  typename CursorList::const_iterator cursor = cursors_.begin();
  int index = 0;

  while (cursor != cursors_.end()) {
    // Has this track ended already?
    if (!cursor->end()) {
      // Track has not ended.
      // Did the next observation occur at this frame?
      if (cursor->time() == time_) {
        // It did. Copy out the point.
        points[index] = cursor->get();
      }
    }

    ++cursor;
    index += 1;
  }
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
MultiViewTimeIterator<T>::MultiViewTimeIterator()
    : views_(), time_(0) {}

template<class T>
MultiViewTimeIterator<T>::MultiViewTimeIterator(
    const MultiviewTrackList<T>& tracks)
    : views_(), time_(0) {
  // Populate iterators.
  for (int view = 0; view < tracks.numViews(); view += 1) {
    views_.push_back(SingleViewTimeIterator<T>(tracks, view));
  }
}

template<class T>
void MultiViewTimeIterator<T>::next() {
  typename IteratorList::iterator view;
  for (view = views_.begin(); view != views_.end(); ++view) {
    view->next();
  }

  time_ += 1;
}

template<class T>
bool MultiViewTimeIterator<T>::end() const {
  typename IteratorList::const_iterator view;
  for (view = views_.begin(); view != views_.end(); ++view) {
    // If any single view iterator hasn't ended, then neither have we.
    if (!view->end()) {
      return false;
    }
  }

  return true;
}

template<class T>
int MultiViewTimeIterator<T>::time() const {
  return time_;
}

template<class T>
void MultiViewTimeIterator<T>::get(
    std::map<int, std::map<int, T> >& features) const {
  features.clear();

  typename IteratorList::const_iterator view;
  int i = 0;

  for (view = views_.begin(); view != views_.end(); ++view) {
    // Extract the features from this view.
    std::map<int, T> subset;
    view->get(subset);

    // Copy them into the multi-view data structure.
    typename std::map<int, T>::const_iterator feature;
    for (feature = subset.begin(); feature != subset.end(); ++feature) {
      features[feature->first][i] = feature->second;
    }

    i += 1;
  }
}

template<class T>
void MultiViewTimeIterator<T>::getView(int view,
                                       std::map<int, T>& points) const {
  views_[view].get(points);
}

template<class T>
const SingleViewTimeIterator<T>& MultiViewTimeIterator<T>::view(
    int view) const {
  return views_[view];
}
