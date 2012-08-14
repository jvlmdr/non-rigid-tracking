////////////////////////////////////////////////////////////////////////////////
// Track

template<class T>
Track<T>::Track() : map_() {}

template<class T>
T& Track<T>::operator[](int x) {
  return map_[x];
}

template<class T>
typename Track<T>::const_iterator Track<T>::find(int x) const {
  return map_.find(x);
}

template<class T>
typename Track<T>::iterator Track<T>::find(int x) {
  return map_.find(x);
}

template<class T>
int Track<T>::size() const {
  return map_.size();
}

template<class T>
bool Track<T>::empty() const {
  return map_.empty();
}

template<class T>
void Track<T>::clear() {
  map_.clear();
}

template<class T>
typename Track<T>::iterator Track<T>::begin() {
  return map_.begin();
}

template<class T>
typename Track<T>::const_iterator Track<T>::begin() const {
  return map_.begin();
}

template<class T>
typename Track<T>::iterator Track<T>::end() {
  return map_.end();
}

template<class T>
typename Track<T>::const_iterator Track<T>::end() const {
  return map_.end();
}

template<class T>
typename Track<T>::reverse_iterator Track<T>::rbegin() {
  return map_.rbegin();
}

template<class T>
typename Track<T>::const_reverse_iterator Track<T>::rbegin() const {
  return map_.rbegin();
}

template<class T>
typename Track<T>::reverse_iterator Track<T>::rend() {
  return map_.rend();
}

template<class T>
typename Track<T>::const_reverse_iterator Track<T>::rend() const {
  return map_.rend();
}

////////////////////////////////////////////////////////////////////////////////
// TrackCursor

template<class T>
TrackCursor_<T>::TrackCursor_() : track(NULL), point() {}

template<class T>
TrackCursor_<T>::TrackCursor_(const Track<T>& track,
                              const Position& point)
    : track(&track), point(point) {}

template<class T>
TrackCursor_<T> TrackCursor_<T>::make(const Track<T>& track) {
  return TrackCursor_(track, track.begin());
}

template<class T>
bool TrackCursor_<T>::end() const {
  return point == track->end();
}
