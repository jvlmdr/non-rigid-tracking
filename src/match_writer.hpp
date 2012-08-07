#ifndef MATCH_WRITER_HPP_
#define MATCH_WRITER_HPP_

#include "match.hpp"
#include "writer.hpp"

class MatchWriter : public Writer<Match> {
  public:
    ~MatchWriter();
    void write(cv::FileStorage& file, const Match& match);
};

#endif
