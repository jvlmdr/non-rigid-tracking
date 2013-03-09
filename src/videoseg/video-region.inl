namespace videoseg {

inline const VideoRegion::FrameList& VideoRegion::frames() const {
  return frames_;
}

inline VideoRegion::FrameList& VideoRegion::frames() {
  return frames_;
}

}
