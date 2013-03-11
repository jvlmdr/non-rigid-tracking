namespace tracking {

inline TrackList::const_iterator TrackList::begin() const {
  return data_.frames().begin();
}

inline TrackList::const_iterator TrackList::end() const {
  return data_.frames().end();
}

inline TrackList::iterator TrackList::begin() {
  return data_.mutable_frames()->begin();
}

inline TrackList::iterator TrackList::end() {
  return data_.mutable_frames()->end();
}

inline int TrackList::numFrames() const {
  return data_.frames().size();
}

inline int TrackList::numPoints() const {
  return num_points_;
}

}
