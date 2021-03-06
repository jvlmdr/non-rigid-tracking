#include <algorithm>
#include <numeric>
#include <glog/logging.h>

template<class T>
MultiviewTrackList<T>::MultiviewTrackList(int num_views)
    : tracks_(), num_views_(num_views), num_frames_(0) {}

template<class T>
MultiviewTrackList<T>::MultiviewTrackList(int num_features, int num_views)
    : tracks_(num_features, MultiviewTrack<T>(num_views)),
      num_views_(num_views),
      num_frames_(0) {}

template<class T>
MultiviewTrackList<T>::MultiviewTrackList()
    : tracks_(), num_views_(0), num_frames_(0) {}

template<class T>
MultiviewTrackList<T>::MultiviewTrackList(const MultiviewTrackList<T>& other)
    : tracks_(other.tracks_),
      num_views_(other.num_views_),
      num_frames_(other.num_frames_) {}

template<class T>
typename MultiviewTrackList<T>::iterator
MultiviewTrackList<T>::begin() {
  return tracks_.begin();
}

template<class T>
typename MultiviewTrackList<T>::iterator
MultiviewTrackList<T>::end() {
  return tracks_.end();
}

template<class T>
typename MultiviewTrackList<T>::const_iterator
MultiviewTrackList<T>::begin() const {
  return tracks_.begin();
}

template<class T>
typename MultiviewTrackList<T>::const_iterator
MultiviewTrackList<T>::end() const {
  return tracks_.end();
}

template<class T>
void MultiviewTrackList<T>::push_back(const MultiviewTrack<T>& x) {
  tracks_.push_back(x);
}

template<class T>
void MultiviewTrackList<T>::pop_back(const MultiviewTrack<T>& x) {
  tracks_.pop_back(x);
}

template<class T>
void MultiviewTrackList<T>::swap(MultiviewTrackList<T>& other) {
  tracks_.swap(other.tracks_);
  std::swap(num_views_, other.num_views_);
  std::swap(num_frames_, other.num_frames_);
}

template<class T>
void swap(MultiviewTrackList<T>& lhs, MultiviewTrackList<T>& rhs) {
  lhs.swap(rhs);
}

template<class T>
void MultiviewTrackList<T>::clear() {
  tracks_.clear();
  num_views_ = 0;
  num_frames_ = 0;
}

template<class T>
MultiviewTrack<T>& MultiviewTrackList<T>::track(int id) {
  return tracks_[id];
}

template<class T>
const MultiviewTrack<T>& MultiviewTrackList<T>::track(int id) const {
  return tracks_[id];
}

template<class T>
MultiviewTrack<T>& MultiviewTrackList<T>::front() {
  return tracks_.front();
}

template<class T>
const MultiviewTrack<T>& MultiviewTrackList<T>::front() const {
  return tracks_.front();
}

template<class T>
MultiviewTrack<T>& MultiviewTrackList<T>::back() {
  return tracks_.back();
}

template<class T>
const MultiviewTrack<T>& MultiviewTrackList<T>::back() const {
  return tracks_.back();
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

template<class T>
int MultiviewTrackList<T>::numImageFeatures() const {
  return std::accumulate(tracks_.begin(), tracks_.end(), 0,
      addMultiviewTrackImageFeatures<T>);
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
  typename MultiviewTrackList<T>::const_iterator multiview_track;

  for (multiview_track = tracks.begin();
       multiview_track != tracks.end();
       ++multiview_track) {
    // Add cursor at beginning of track for given view.
    cursors_.push_back(TrackIterator<T>(multiview_track->view(view_)));
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
