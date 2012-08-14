#include "read_lines.hpp"
#include <fstream>
#include <iostream>

bool readLines(const std::string& filename,
               std::vector<std::string>& lines) {
  // Attempt to open file.
  std::ifstream ifs;
  ifs.open(filename.c_str());
  if (!ifs.is_open()) {
    return false;
  }

  lines.clear();
  std::string line;

  while (std::getline(ifs, line)) {
    lines.push_back(std::string());
    lines.back().swap(line);
  }

  return true;
}
