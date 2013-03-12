namespace videoseg {

inline bool RegionBefore::operator()(
    const VideoSegmentation::Frame::Region& lhs,
    int rhs) const {
  return (lhs.id() < rhs);
}

inline bool RegionBefore::operator()(
    int lhs,
    const VideoSegmentation::Frame::Region& rhs) const {
  return (lhs < rhs.id());
}

inline bool regionBefore(const VideoSegmentation::Frame::Region& lhs,
                         const VideoSegmentation::Frame::Region& rhs) {
  return (lhs.id() < rhs.id());
}

}
