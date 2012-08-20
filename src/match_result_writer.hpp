#ifndef MATCH_RESULT_WRITER_HPP_
#define MATCH_RESULT_WRITER_HPP_

#include "match_result.hpp"
#include "writer.hpp"

// Writes a match result to a file.
class MatchResultWriter : public Writer<MatchResult> {
  public:
    ~MatchResultWriter();
    void write(cv::FileStorage& file, const MatchResult& result);
};

#endif
