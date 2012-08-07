#ifndef MATCH_READER_HPP_
#define MATCH_READER_HPP_

#include "match.hpp"
#include "reader.hpp"

class MatchReader : public Reader<Match> {
  public:
    ~MatchReader();
    void read(const cv::FileNode& node, Match& match);
};

#endif
