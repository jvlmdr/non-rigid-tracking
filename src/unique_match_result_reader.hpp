#ifndef UNIQUE_MATCH_RESULT_READER_HPP_
#define UNIQUE_MATCH_RESULT_READER_HPP_

#include "unique_match_result.hpp"
#include "reader.hpp"

class UniqueMatchResultReader : public Reader<UniqueMatchResult> {
  public:
    ~UniqueMatchResultReader();
    bool read(const cv::FileNode& node, UniqueMatchResult& result);
};

#endif
