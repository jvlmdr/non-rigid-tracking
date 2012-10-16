#ifndef UNIQUE_MATCH_RESULT_WRITER_HPP_
#define UNIQUE_MATCH_RESULT_WRITER_HPP_

#include "unique_match_result.hpp"
#include "writer.hpp"

// Writes a match result to a file.
class UniqueMatchResultWriter : public Writer<UniqueMatchResult> {
  public:
    ~UniqueMatchResultWriter();
    void write(cv::FileStorage& file, const UniqueMatchResult& result);
};

#endif
