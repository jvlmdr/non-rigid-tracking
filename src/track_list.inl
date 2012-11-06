#include <iostream>
#include <numeric>
#include <limits>
#include <boost/bind.hpp>

////////////////////////////////////////////////////////////////////////////////
// TrackList

template<class T>
TrackList<T>::TrackList() : list_() {}

template<class T>
TrackList<T>::TrackList(int size) : list_(size, Track<T>()) {}

template<class T>
int firstFrame(const Track<T>& track) {
  return track.begin()->first;
}

template<class T>
T min(const T& a, const T& b) {
  return std::min(a, b);
}

// Returns the first frame in the track.
template<class T>
int TrackList<T>::findFirstFrame() const {
  std::vector<int> indices;
  std::transform(list_.begin(), list_.end(), std::back_inserter(indices),
      firstFrame<T>);

  return std::accumulate(indices.begin(), indices.end(),
      std::numeric_limits<int>::max(), min<int>);
}

// Returns the number of points in all tracks.
template<class T>
int TrackList<T>::countPoints() const {
  std::vector<int> counts;

  std::transform(list_.begin(), list_.end(), std::back_inserter(counts),
      boost::bind(&Track<T>::size, _1));

  return std::accumulate(counts.begin(), counts.end(), int(0));
}

template<class T>
Track<T>& TrackList<T>::operator[](int n) {
  return list_[n];
}

template<class T>
const Track<T>& TrackList<T>::operator[](int n) const {
  return list_[n];
}

template<class T>
void TrackList<T>::push_back(const Track<T>& x) {
  list_.push_back(x);
}

template<class T>
Track<T>& TrackList<T>::back() {
  return list_.back();
}

template<class T>
const Track<T>& TrackList<T>::back() const {
  return list_.back();
}

template<class T>
Track<T>& TrackList<T>::front() {
  return list_.front();
}

template<class T>
const Track<T>& TrackList<T>::front() const {
  return list_.front();
}

template<class T>
int TrackList<T>::size() const {
  return list_.size();
}

template<class T>
bool TrackList<T>::empty() const {
  return list_.empty();
}

template<class T>
void TrackList<T>::clear() {
  list_.clear();
}

template<class T>
void TrackList<T>::swap(TrackList<T>& other) {
  other.list_.swap(list_);
}

template<class T>
void swap(TrackList<T>& lhs, TrackList<T>& rhs) {
  return lhs.swap(rhs);
}

template<class T>
int TrackList<T>::numImageFeatures() const {
  return std::accumulate(list_.begin(), list_.end(), 0, addTrackSize<T>);
}

template<class T>
typename TrackList<T>::iterator TrackList<T>::begin() {
  return list_.begin();
}

template<class T>
typename TrackList<T>::const_iterator TrackList<T>::begin() const {
  return list_.begin();
}

template<class T>
typename TrackList<T>::iterator TrackList<T>::end() {
  return list_.end();
}

template<class T>
typename TrackList<T>::const_iterator TrackList<T>::end() const {
  return list_.end();
}

////////////////////////////////////////////////////////////////////////////////
// TrackListTimeIterator

// Initializes at start of tracks.
template<class T>
TrackListTimeIterator<T>::TrackListTimeIterator(const TrackList<T>& tracks)
    : cursors_(), t_(0) {
  // Iterate through tracks.
  typename TrackList<T>::const_iterator track = tracks.begin();
  // Monitor track index.
  int i = 0;

  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Add a cursor for each non-empty track.
    if (!track->empty()) {
      cursors_[i] = TrackIterator<T>(*track);
    }

    i += 1;
  }
}

template<class T>
TrackListTimeIterator<T>::TrackListTimeIterator(const TrackList<T>& tracks,
                                                int t)
    : cursors_(), t_(t) {
  // Iterate through tracks.
  typename TrackList<T>::const_iterator track = tracks.begin();
  // Monitor track index.
  int i = 0;

  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Add a cursor for each non-empty track.
    if (!track->empty()) {
      cursors_[i] = TrackIterator<T>(*track, t);
    }

    i += 1;
  }
}

// Copy constructor.
template<class T>
TrackListTimeIterator<T>::TrackListTimeIterator(
    const TrackListTimeIterator& rhs)
    : cursors_(rhs.cursors_), t_(rhs.t_) {}

// Pre-increment.
// Updates the list of points in this frame and advances the cursor.
template<class T>
TrackListTimeIterator<T>& TrackListTimeIterator<T>::operator++() {
  // Iterate through tracks.
  typename CursorList::iterator cursor = cursors_.begin();
  while (cursor != cursors_.end()) {
    bool remove = false;

    // Get space-time point.
    int t = cursor->second.time();

    // If track appeared in this frame, advance the cursor.
    if (t == t_) {
      cursor->second.next();
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
TrackListTimeIterator<T> TrackListTimeIterator<T>::operator++(int) {
  TrackListTimeIterator<T> result(*this);
  ++(*this);
  return result;
}

template<class T>
void TrackListTimeIterator<T>::seekToStart() {
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
int TrackListTimeIterator<T>::t() const {
  return t_;
}

template<class T>
void TrackListTimeIterator<T>::getPoints(Points& points) const {
  // Iterate through tracks.
  typename CursorList::const_iterator cursor;

  for (cursor = cursors_.begin(); cursor != cursors_.end(); ++cursor) {
    // Get space-time point.
    int i = cursor->first;
    int t = cursor->second.time();
    const T& x = cursor->second.get();

    // If track appeared in this frame, add to points.
    if (t == t_) {
      points[i] = x;
    }
  }
}

// Returns true if this is the last frame.
template<class T>
bool TrackListTimeIterator<T>::end() const {
  return cursors_.empty();
}
