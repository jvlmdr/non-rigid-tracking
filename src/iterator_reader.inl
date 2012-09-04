template<class T, class OutputIterator>
bool readSequence(const cv::FileNode& node,
                  Reader<T>& reader,
                  OutputIterator output) {
  // Check node is not empty.
  if (node.type() == cv::FileNode::NONE) {
    LOG(WARNING) << "Empty file node";
    return false;
  }

  // Check node is a map.
  if (node.type() != cv::FileNode::MAP) {
    LOG(WARNING) << "Expected file node to be a map";
    return false;
  }

  const cv::FileNode& child = node["list"];

  // Check that child exists.
  if (child.type() == cv::FileNode::NONE) {
    LOG(WARNING) << "Empty file node";
    return false;
  }

  // Check that child is a sequence.
  if (child.type() != cv::FileNode::SEQ) {
    LOG(WARNING) << "Expected file node to be a sequence";
    return false;
  }

  for (cv::FileNodeIterator it = child.begin(); it != child.end(); ++it) {
    T x;
    if (!reader.read(*it, x)) {
      return false;
    }
    *output++ = x;
  }

  return true;
}
