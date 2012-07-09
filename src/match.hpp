#ifndef MATCH_HPP_
#define MATCH_HPP_

#include <utility>
#include <vector>
#include <string>

typedef std::pair<int, int> Match;
typedef std::vector<Match> MatchList;

bool saveMatches(const std::string& filename, const MatchList& matches);

#endif
