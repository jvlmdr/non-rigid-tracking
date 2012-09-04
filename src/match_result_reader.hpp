#ifndef MATCH_RESULT_READER_HPP_
#define MATCH_RESULT_READER_HPP_

#include "match_result.hpp"
#include "reader.hpp"

class MatchResultReader : public Reader<MatchResult> {
  public:
    ~MatchResultReader();
    bool read(const cv::FileNode& node, MatchResult& result);
};

#endif
