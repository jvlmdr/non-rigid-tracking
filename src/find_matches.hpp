#include "match_result.hpp"
#include "descriptor.hpp"
#include <vector>

// Matching in both directions returns directed matches.
struct DirectedMatch {
  int index;
  double distance;

  DirectedMatch();
  DirectedMatch(int index, double distance);
};

// And in fact, multiple directed matches.
typedef std::vector<DirectedMatch> DirectedMatchList;

// Finds all matches for each point in the other set.
// Limited by max_num_matches and max_relative_distance.
// Outputs a list of matched points for each point.
void matchBothDirections(const std::vector<Descriptor>& points1,
                         const std::vector<Descriptor>& points2,
                         std::vector<DirectedMatchList>& forward_matches,
                         std::vector<DirectedMatchList>& reverse_matches,
                         int max_num_matches,
                         double max_relative_distance,
                         bool use_flann);

// Finds the reciprocal matches.
void intersectionOfMatchLists(
    const std::vector<DirectedMatchList>& forward_matches,
    const std::vector<DirectedMatchList>& reverse_matches,
    std::vector<MatchResult>& matches);

// Combines all matches.
void unionOfMatchLists(
    const std::vector<DirectedMatchList>& forward_matches,
    const std::vector<DirectedMatchList>& reverse_matches,
    std::vector<MatchResult>& matches);
