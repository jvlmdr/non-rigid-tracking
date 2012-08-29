#include "match_result.hpp"
#include "descriptor.hpp"
#include <vector>

struct DirectedMatchResult {
  int match;
  double dist;
  double second_dist;
};

void matchBothDirections(const std::vector<Descriptor>& list1,
                         const std::vector<Descriptor>& list2,
                         std::vector<DirectedMatchResult>& forward_matches,
                         std::vector<DirectedMatchResult>& reverse_matches,
                         bool use_flann);

void intersectionOfMatches(
    const std::vector<DirectedMatchResult>& forward_matches,
    const std::vector<DirectedMatchResult>& reverse_matches,
    std::vector<MatchResult>& matches);

void unionOfMatches(const std::vector<DirectedMatchResult>& forward_matches,
                    const std::vector<DirectedMatchResult>& reverse_matches,
                    std::vector<MatchResult>& matches);

typedef std::vector<Descriptor> DescriptorBag;

void matchBagsBothDirections(const std::vector<DescriptorBag>& bags1,
                             const std::vector<DescriptorBag>& bags2,
                             std::vector<DirectedMatchResult>& forward_matches,
                             std::vector<DirectedMatchResult>& reverse_matches,
                             bool use_flann);
