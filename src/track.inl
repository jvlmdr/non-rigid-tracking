////////////////////////////////////////////////////////////////////////////////
// Track

template<class T>
Track_<T>::Track_() : map_() {}

template<class T>
T& Track_<T>::operator[](int x) {
  return map_[x];
}

template<class T>
typename Track_<T>::const_iterator Track_<T>::find(int x) const {
  return map_.find(x);
}

template<class T>
typename Track_<T>::iterator Track_<T>::find(int x) {
  return map_.find(x);
}

template<class T>
int Track_<T>::size() const {
  return map_.size();
}

template<class T>
bool Track_<T>::empty() const {
  return map_.empty();
}

template<class T>
void Track_<T>::clear() {
  map_.clear();
}

template<class T>
typename Track_<T>::iterator Track_<T>::begin() {
  return map_.begin();
}

template<class T>
typename Track_<T>::const_iterator Track_<T>::begin() const {
  return map_.begin();
}

template<class T>
typename Track_<T>::iterator Track_<T>::end() {
  return map_.end();
}

template<class T>
typename Track_<T>::const_iterator Track_<T>::end() const {
  return map_.end();
}

template<class T>
typename Track_<T>::reverse_iterator Track_<T>::rbegin() {
  return map_.rbegin();
}

template<class T>
typename Track_<T>::const_reverse_iterator Track_<T>::rbegin() const {
  return map_.rbegin();
}

template<class T>
typename Track_<T>::reverse_iterator Track_<T>::rend() {
  return map_.rend();
}

template<class T>
typename Track_<T>::const_reverse_iterator Track_<T>::rend() const {
  return map_.rend();
}

////////////////////////////////////////////////////////////////////////////////
// TrackCursor

template<class T>
TrackCursor_<T>::TrackCursor_() : track(NULL), point() {}

template<class T>
TrackCursor_<T>::TrackCursor_(const Track_<T>& track,
                              const Position& point)
    : track(&track), point(point) {}

template<class T>
TrackCursor_<T> TrackCursor_<T>::make(const Track_<T>& track) {
  return TrackCursor_(track, track.begin());
}

template<class T>
bool TrackCursor_<T>::end() const {
  return point == track->end();
}
