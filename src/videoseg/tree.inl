namespace videoseg {

inline bool SegmentationNode::isLeaf() const {
  return static_cast<bool>(leaf_);
}

inline const SegmentationNode::Leaf& SegmentationNode::leaf() const {
  return *leaf_;
}

inline SegmentationNode::Leaf& SegmentationNode::leaf() {
  return *leaf_;
}

inline const SegmentationNode::Interior& SegmentationNode::interior() const {
  return *interior_;
}

inline SegmentationNode::Interior& SegmentationNode::interior() {
  return *interior_;
}

} // namespace videoseg
