#ifndef MATCH_HPP_
#define MATCH_HPP_

#include <utility>
#include <vector>
#include <string>

typedef std::pair<int, int> Match;
typedef std::vector<Match> MatchList;

// Saves a list of matches to a file.
bool saveMatches(const std::string& filename, const MatchList& matches);

// Loads a list of matches from a file.
bool loadMatches(const std::string& filename, MatchList& matches);

#endif
