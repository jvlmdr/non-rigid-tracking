#include "unique_match_result.hpp"
#include "descriptor.hpp"
#include "classifier.hpp"
#include <vector>
#include <map>
#include <deque>

// The result of a unique match includes the distance to the second-best.
struct UniqueQueryResult {
  int index;
  double distance;
  double next_best;

  UniqueQueryResult();
  UniqueQueryResult(int index, double distance, double next_best);
};

void convertUniqueQueryResultsToMatches(
    const std::vector<UniqueQueryResult>& directed,
    std::vector<UniqueMatchResult>& undirected,
    bool forward);

// Find the single best match for each point.
// Returns also the distance to the second-best match.
void findUniqueMatchesUsingClassifiers(
    const std::deque<Classifier>& classifiers,
    const std::deque<Descriptor>& points,
    std::vector<UniqueQueryResult>& matches);

// Find the single best match for each point.
// Returns also the distance to the second-best match.
void findUniqueMatchesUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::vector<UniqueQueryResult>& matches,
    bool use_flann);

// If we're matching in both directions, avoid copying.
void findUniqueMatchesInBothDirectionsUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::vector<UniqueQueryResult>& forward_matches,
    std::vector<UniqueQueryResult>& reverse_matches,
    bool use_flann);
