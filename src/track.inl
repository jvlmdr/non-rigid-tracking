#include <glog/logging.h>

////////////////////////////////////////////////////////////////////////////////
// Track

template<class T>
Track<T>::Track() : map_() {}

template<class T>
void Track<T>::resetRange(int first, int last) {
  for (int t = first; t <= last; t += 1) {
    map_[t];
  }
}

template<class T>
T& Track<T>::operator[](int x) {
  return map_[x];
}

template<class T>
std::pair<typename Track<T>::iterator, bool>
Track<T>::insert(const value_type& x) {
  return map_.insert(x);
}

template<class T>
typename Track<T>::iterator
Track<T>::insert(iterator position, const value_type& x) {
  return map_.insert(position, x);
}

template<class T>
template<class InputIterator>
void Track<T>::insert(InputIterator first, InputIterator last) {
  map_.insert(first, last);
}

template<class T>
typename Track<T>::iterator Track<T>::find(int x) {
  return map_.find(x);
}

template<class T>
typename Track<T>::const_iterator Track<T>::find(int x) const {
  return map_.find(x);
}

template<class T>
void Track<T>::erase(iterator position) {
  map_.erase(position);
}

template<class T>
typename Track<T>::iterator Track<T>::lower_bound(int x) {
  return map_.lower_bound(x);
}

template<class T>
typename Track<T>::const_iterator Track<T>::lower_bound(int x) const {
  return map_.lower_bound(x);
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
void Track<T>::swap(Track<T>& other) {
  map_.swap(other.map_);
}

template<class T>
void swap(Track<T>& lhs, Track<T>& rhs) {
  lhs.swap(rhs);
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

template<class T>
int addTrackSize(int x, const Track<T>& track) {
  return x + track.size();
}

////////////////////////////////////////////////////////////////////////////////
// TrackIterator

template<class T>
TrackIterator<T>::TrackIterator() : track_(NULL), position_() {}

template<class T>
TrackIterator<T>::TrackIterator(const Track<T>& track)
    : track_(&track), position_(track.begin()) {}

template<class T>
TrackIterator<T>::TrackIterator(const Track<T>& track, int time)
    : track_(&track), position_() {
  // Find the last component whose key is less than or equal to time.
  position_ = track_->lower_bound(time);
}

template<class T>
TrackIterator<T>::TrackIterator(const TrackIterator<T>& other)
    : track_(other.track_), position_(other.position_) {}

template<class T>
void TrackIterator<T>::next() {
  CHECK_NOTNULL(track_);
  ++position_;
}

template<class T>
void TrackIterator<T>::previous() {
  CHECK_NOTNULL(track_);
  --position_;
}

template<class T>
bool TrackIterator<T>::end() const {
  CHECK_NOTNULL(track_);
  return (position_ == track_->end());
}

template<class T>
bool TrackIterator<T>::begin() const {
  CHECK_NOTNULL(track_);
  return (position_ == track_->begin());
}

template<class T>
const T& TrackIterator<T>::get() const {
  CHECK_NOTNULL(track_);
  return position_->second;
}

template<class T>
int TrackIterator<T>::time() const {
  CHECK_NOTNULL(track_);
  return position_->first;
}
