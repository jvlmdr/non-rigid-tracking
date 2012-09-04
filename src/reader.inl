#include <glog/logging.h>

template<class T, class X>
bool read(const cv::FileNode& node, X& x) {
  if (node.type() == cv::FileNode::NONE) {
    return false;
  }

  x = static_cast<T>(node);

  return true;
}

template<class T>
bool load(const std::string& filename, T& x, Reader<T>& reader) {
  // Try to open file.
  cv::FileStorage file(filename, cv::FileStorage::READ);
  if (!file.isOpened()) {
    LOG(WARNING) << "Could not open `" << filename << "' for reading";
    return false;
  }

  if (!reader.read(file.root(), x)) {
    return false;
  }

  return true;
}
