#include "match.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/bind.hpp>

////////////////////////////////////////////////////////////////////////////////
// Saving

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

////////////////////////////////////////////////////////////////////////////////
// Loading

namespace {

// Reads a single match from a file.
Match readMatchFromFile(const cv::FileNode& node) {
  cv::FileNodeIterator it = node.begin();

  int first = int(*it);
  ++it;
  int second = int(*it);

  return Match(first, second);
}

}

// Loads a list of matches from a file.
bool loadMatches(const std::string& filename, MatchList& matches) {
  // Open file to save tracks.
  cv::FileStorage file(filename, cv::FileStorage::READ);
  if (!file.isOpened()) {
    std::cerr << "could not open file " << filename << std::endl;
    return false;
  }

  cv::FileNode list = file["matches"];
  std::transform(list.begin(), list.end(), std::back_inserter(matches),
      readMatchFromFile);

  return true;
}
