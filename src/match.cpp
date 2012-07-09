#include "match.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/bind.hpp>

namespace {

// Writes a single match to a file.
bool writeMatch(cv::FileStorage& file, const Match& match) {
  file << "[:";
  file << match.first;
  file << match.second;
  file << "]";

  return true;
}

}

// Saves a list of matches to a file.
bool saveMatches(const std::string& filename, const MatchList& matches) {
  // Open file to save tracks.
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  file << "matches" << "[";
  std::for_each(matches.begin(), matches.end(),
      boost::bind(writeMatch, boost::ref(file), _1));
  file << "]";

  return true;
}
