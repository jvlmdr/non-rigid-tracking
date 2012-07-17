#include <iostream>
#include <numeric>
#include <limits>

template<class T>
Write<T>::~Write() {}

template<class T>
Read<T>::~Read() {}

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
bool TrackList_<T>::save(const std::string& filename,
                         Write<T>& write_point) const {
  // Open file to save tracks.
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  // Save each frame out to file.
  file << "tracks" << "[";

  // Iterate through frames.
  FrameIterator_<T> frame(*this);
  int t = 0;

  while (!frame.end()) {
    file << "[";

    // Read out points.
    typedef typename FrameIterator_<T>::Points Points;
    Points points;
    frame.getPoints(points);

    // Write each point out.
    typename Points::const_iterator point;
    for (point = points.begin(); point != points.end(); ++point) {
      file << "{";
      file << "id" << point->first;
      file << "point";
      write_point(file, point->second);
      file << "}";
    }

    file << "]";
    frame++;
    t += 1;
  }

  file << "]";

  return true;
}

template<class T>
bool TrackList_<T>::load(const std::string& filename, Read<T>& read_point) {
  // Open file to read tracks.
  cv::FileStorage file(filename, cv::FileStorage::READ);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  // Map from integer IDs to track indices.
  typedef std::map<int, int> Lookup;
  Lookup lookup;

  cv::FileNode frames = file["tracks"];
  cv::FileNodeIterator frame;
  int t = 0;

  for (frame = frames.begin(); frame != frames.end(); ++frame) {
    cv::FileNode points = (*frame);

    cv::FileNodeIterator node;
    for (node = points.begin(); node != points.end(); ++node) {
      // Read point data from file.
      int id = (int)((*node)["id"]);
      T point;
      read_point((*node)["point"], point);

      // Check whether track exists.
      Lookup::iterator e = lookup.find(id);
      Track_<T>* track;

      if (e == lookup.end()) {
        // Does not exist. Create new track.
        list_.push_back(Track_<T>());
        track = &list_.back();
        lookup[id] = list_.size() - 1;
      } else {
        // Add to existing track.
        track = &list_[e->second];
      }

      // Add the point at this time instant.
      (*track)[t] = point;
    }

    t += 1;
  }

  return true;

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
