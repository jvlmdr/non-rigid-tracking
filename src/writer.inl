#include <glog/logging.h>

// Saves anything which has an appropriate Writer.
template<class T>
bool save(const std::string& filename, const T& x, Writer<T>& writer) {
  // Try to open file.
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    LOG(WARNING) << "Could not open `" << filename << "' for writing";
    return false;
  }

  writer.write(file, x);

  return true;
}
