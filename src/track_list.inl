#include <iostream>
#include <numeric>
#include <limits>
#include <boost/bind.hpp>

////////////////////////////////////////////////////////////////////////////////
// TrackList

template<class T>
TrackList_<T>::TrackList_() : list_() {}

template<class T>
TrackList_<T>::TrackList_(int size) : list_(size) {}

template<class T>
int firstFrame(const Track_<T>& track) {
  return track.begin()->first;
}

template<class T>
T min(const T& a, const T& b) {
  return std::min(a, b);
}

// Returns the first frame in the track.
template<class T>
int TrackList_<T>::findFirstFrame() const {
  std::vector<int> indices;
  std::transform(list_.begin(), list_.end(), std::back_inserter(indices),
      firstFrame<T>);

  return std::accumulate(indices.begin(), indices.end(),
      std::numeric_limits<int>::max(), min<int>);
}

// Returns the number of points in all tracks.
template<class T>
int TrackList_<T>::countPoints() const {
  std::vector<int> counts;

  std::transform(list_.begin(), list_.end(), std::back_inserter(counts),
      boost::bind(&Track_<T>::size, _1));

  return std::accumulate(counts.begin(), counts.end(), int(0));
}

template<class T>
Track_<T>& TrackList_<T>::operator[](int n) {
  return list_[n];
}

template<class T>
const Track_<T>& TrackList_<T>::operator[](int n) const {
  return list_[n];
}

template<class T>
void TrackList_<T>::push_back(const Track_<T>& x) {
  list_.push_back(x);
}

template<class T>
Track_<T>& TrackList_<T>::back() {
  return list_.back();
}

template<class T>
const Track_<T>& TrackList_<T>::back() const {
  return list_.back();
}

template<class T>
Track_<T>& TrackList_<T>::front() {
  return list_.front();
}

template<class T>
const Track_<T>& TrackList_<T>::front() const {
  return list_.front();
}

template<class T>
int TrackList_<T>::size() const {
  return list_.size();
}

template<class T>
bool TrackList_<T>::empty() const {
  return list_.empty();
}

template<class T>
void TrackList_<T>::clear() {
  list_.clear();
}

template<class T>
void TrackList_<T>::swap(TrackList_<T>& other) {
  other.list_.swap(list_);
}

template<class T>
typename TrackList_<T>::iterator TrackList_<T>::begin() {
  return list_.begin();
}

template<class T>
typename TrackList_<T>::const_iterator TrackList_<T>::begin() const {
  return list_.begin();
}

template<class T>
typename TrackList_<T>::iterator TrackList_<T>::end() {
  return list_.end();
}

template<class T>
typename TrackList_<T>::const_iterator TrackList_<T>::end() const {
  return list_.end();
}

template<class T>
typename TrackList_<T>::reverse_iterator TrackList_<T>::rbegin() {
  return list_.rbegin();
}

template<class T>
typename TrackList_<T>::const_reverse_iterator TrackList_<T>::rbegin() const {
  return list_.rbegin();
}

template<class T>
typename TrackList_<T>::reverse_iterator TrackList_<T>::rend() {
  return list_.rend();
}

template<class T>
typename TrackList_<T>::const_reverse_iterator TrackList_<T>::rend() const {
  return list_.rend();
}

////////////////////////////////////////////////////////////////////////////////
// FrameIterator_

// Initializes at start of tracks.
template<class T>
FrameIterator_<T>::FrameIterator_(const TrackList_<T>& tracks)
    : cursors_(), t_(0) {
  // Iterate through tracks.
  typename TrackList_<T>::const_iterator track = tracks.begin();
  // Monitor track index.
  int i = 0;

  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Add a cursor for each non-empty track.
    if (!track->empty()) {
      cursors_[i] = TrackCursor_<T>::make(*track);
    }

    i += 1;
  }
}

// Copy constructor.
template<class T>
FrameIterator_<T>::FrameIterator_(const FrameIterator_& rhs)
    : cursors_(rhs.cursors_), t_(rhs.t_) {}

// Pre-increment.
// Updates the list of points in this frame and advances the cursor.
template<class T>
FrameIterator_<T>& FrameIterator_<T>::operator++() {
  // Iterate through tracks.
  typename CursorList::iterator cursor = cursors_.begin();
  while (cursor != cursors_.end()) {
    bool remove = false;

    // Get space-time point.
    int t = cursor->second.point->first;

    // If track appeared in this frame, advance the cursor.
    if (t == t_) {
      ++cursor->second.point;
      // If the cursor reached the end of the track, remove it.
      if (cursor->second.end()) {
        remove = true;
      }
    }

    if (remove) {
      cursors_.erase(cursor++);
    } else {
      ++cursor;
    }
  }

  t_ += 1;

  return *this;
}

// Post-increment.
template<class T>
FrameIterator_<T> FrameIterator_<T>::operator++(int) {
  FrameIterator_<T> result(*this);
  ++(*this);
  return result;
}

template<class T>
void FrameIterator_<T>::seekToStart() {
  bool found = false;

  while (!found && !end()) {
    Points points;
    getPoints(points);

    found = !points.empty();

    if (!found) {
      ++(*this);
    }
  }
}

template<class T>
int FrameIterator_<T>::t() const {
  return t_;
}

template<class T>
void FrameIterator_<T>::getPoints(Points& points) const {
  // Iterate through tracks.
  typename CursorList::const_iterator cursor;

  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // Get space-time point.
    int i = cursor->first;
    int t = cursor->second.point->first;
    const T& x = cursor->second.point->second;

    // If track appeared in this frame, add to points.
    if (t == t_) {
      points[i] = x;
    }
  }
}

// Returns true if this is the last frame.
template<class T>
bool FrameIterator_<T>::end() const {
  return cursors_.empty();
}
