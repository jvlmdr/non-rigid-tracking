#include "match_result.hpp"
#include "match.hpp"
#include "descriptor.hpp"
#include "classifier.hpp"
#include <vector>
#include <deque>

// Matching in both directions returns directed matches.
struct QueryResult {
  int index;
  double distance;

  QueryResult();
  QueryResult(int index, double distance);
};

// And in fact, multiple directed matches.
typedef std::vector<QueryResult> QueryResultList;

void convertQueryResultListsToMatches(
    const std::deque<QueryResultList>& directed,
    std::vector<MatchResult>& undirected,
    bool forward);

// Finds all matches for each point in the other set.
// Limited by max_num_matches and max_relative_distance.
// Outputs a list of matched points for each point.
void findMatchesUsingClassifiers(const std::deque<Classifier>& classifiers,
                                 const std::deque<Descriptor>& points,
                                 std::deque<QueryResultList>& matches,
                                 int max_num_matches);

void findMatchesUsingClassifier(const Classifier& classifier,
                                const std::deque<Descriptor>& points,
                                std::vector<QueryResult>& matches,
                                size_t max_num_matches);

// Finds all matches for each point in the other set.
// Limited by max_num_matches and max_relative_distance.
// Outputs a list of matched points for each point.
void findMatchesUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::deque<QueryResultList>& matches,
    int max_num_matches,
    double max_relative_distance,
    bool use_flann);

// Bundle forward and reverse matching together when using Euclidean distance.
// Saves converting to cv::Mats twice.
void findMatchesInBothDirectionsUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::deque<QueryResultList>& forward_matches,
    std::deque<QueryResultList>& reverse_matches,
    int max_num_matches,
    double max_relative_distance,
    bool use_flann);
