#ifndef READ_LINES_HPP_
#define READ_LINES_HPP_

#include <vector>
#include <string>
#include "parser.hpp"

bool readLines(const std::string& filename,
               std::vector<std::string>& lines);

template<class T>
bool readLinesOf(const std::string& filename,
                 std::vector<T>& values,
                 Parser<T>& parser);

#include "read_lines.inl"

#endif
