#include "unique_match_result.hpp"
#include "descriptor.hpp"
#include <vector>

// The result of a unique match includes the distance to the second-best.
struct UniqueDirectedMatch {
  int index;
  double distance;
  double next_best;

  UniqueDirectedMatch();
  UniqueDirectedMatch(int index, double distance, double next_best);
};

// Find the single best match for each point.
// Returns also the distance to the second-best match.
void matchUniqueBothDirections(
    const std::vector<Descriptor>& points1,
    const std::vector<Descriptor>& points2,
    std::vector<UniqueDirectedMatch>& forward_matches,
    std::vector<UniqueDirectedMatch>& reverse_matches,
    bool use_flann);

void intersectionOfUniqueMatches(
    const std::vector<UniqueDirectedMatch>& forward_matches,
    const std::vector<UniqueDirectedMatch>& reverse_matches,
    std::vector<UniqueMatchResult>& matches);

void unionOfUniqueMatches(
    const std::vector<UniqueDirectedMatch>& forward_matches,
    const std::vector<UniqueDirectedMatch>& reverse_matches,
    std::vector<UniqueMatchResult>& matches);
