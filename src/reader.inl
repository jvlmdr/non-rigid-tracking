#include <glog/logging.h>

template<class T>
bool load(const std::string& filename, T& x, Reader<T>& reader) {
  // Try to open file.
  cv::FileStorage file(filename, cv::FileStorage::READ);
  if (!file.isOpened()) {
    LOG(WARNING) << "Could not open `" << filename << "' for reading";
    return false;
  }

  reader.read(file["root"], x);

  return true;
}
