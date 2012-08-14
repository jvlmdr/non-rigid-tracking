#include <glog/logging.h>

template<class T>
bool readLinesOf(const std::string& filename,
                 std::vector<T>& values,
                 Parser<T>& parser) {
  // Read strings.
  std::vector<std::string> lines;
  bool ok = readLines(filename, lines);
  if (!ok) {
    LOG(WARNING) << "Could not read lines of file `" << filename << "'";
    return false;
  }

  // Convert.
  for (std::vector<std::string>::const_iterator line = lines.begin();
       line != lines.end();
       ++line) {
    T value;
    ok = parser.parse(*line, value);
    if (!ok) {
      LOG(WARNING) << "Could not parse `" << *line << "'";
      return false;
    }

    values.push_back(value);
  }

  return true;
}
