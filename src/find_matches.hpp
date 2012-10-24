#include "match_result.hpp"
#include "descriptor.hpp"
#include "classifier.hpp"
#include <vector>
#include <deque>

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
void findMatchesUsingClassifiers(const std::deque<Classifier>& classifiers,
                                 const std::deque<Descriptor>& points,
                                 std::deque<DirectedMatchList>& matches,
                                 int max_num_matches);

void findMatchesUsingClassifier(const Classifier& classifier,
                                const std::deque<Descriptor>& points,
                                DirectedMatchList& matches,
                                size_t max_num_matches);

// Bundle forward and reverse matching together when using Euclidean distance.
// Saves converting to cv::Mats twice.
//
// Finds all matches for each point in the other set.
// Limited by max_num_matches and max_relative_distance.
// Outputs a list of matched points for each point.
void findMatchesInBothDirectionsUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::deque<DirectedMatchList>& forward_matches,
    std::deque<DirectedMatchList>& reverse_matches,
    int max_num_matches,
    double max_relative_distance,
    bool use_flann);

// Finds the reciprocal matches.
void intersectionOfMatchLists(
    const std::deque<DirectedMatchList>& forward_matches,
    const std::deque<DirectedMatchList>& reverse_matches,
    std::vector<MatchResult>& matches);

// Combines all matches.
void unionOfMatchLists(
    const std::deque<DirectedMatchList>& forward_matches,
    const std::deque<DirectedMatchList>& reverse_matches,
    std::vector<MatchResult>& matches);
