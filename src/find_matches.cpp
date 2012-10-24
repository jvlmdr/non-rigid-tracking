#include "find_matches.hpp"
#include <set>
#include <glog/logging.h>
#include <algorithm>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "find_matches_util.hpp"

DirectedMatch::DirectedMatch() : index(-1), distance(-1) {}

DirectedMatch::DirectedMatch(int index, double distance)
    : index(index), distance(distance) {}

////////////////////////////////////////////////////////////////////////////////

class CompareDirectedMatches {
  public:
    CompareDirectedMatches(bool ascending);

    bool operator()(const DirectedMatch& lhs, const DirectedMatch& rhs) const;

  private:
    bool ascending_;
};

CompareDirectedMatches::CompareDirectedMatches(bool ascending)
    : ascending_(ascending) {}

bool CompareDirectedMatches::operator()(const DirectedMatch& lhs,
                                        const DirectedMatch& rhs) const {
  if (ascending_) {
    return lhs.distance < rhs.distance;
  } else {
    return lhs.distance > rhs.distance;
  }
}

void findMatchesUsingClassifier(const Classifier& classifier,
                                const std::deque<Descriptor>& points,
                                DirectedMatchList& matches,
                                size_t max_num_matches) {
  matches.clear();

  // Use a heap to keep track of the worst (sort highest to lowest by distance).
  CompareDirectedMatches compare(true);

  std::deque<Descriptor>::const_iterator point;
  int index = 0;

  for (point = points.begin(); point != points.end(); ++point) {
    double distance = std::exp(-classifier.score(*point));

    // If we haven't reached the maximum number of matches or this match is
    // better than the worst, add it to the heap.
    if (matches.size() < max_num_matches ||
        distance < matches.front().distance) {
      matches.push_back(DirectedMatch(index, distance));
      std::push_heap(matches.begin(), matches.end(), compare);
    }

    // If there are too many elements, remove from the heap.
    if (matches.size() >= max_num_matches) {
      std::pop_heap(matches.begin(), matches.end(), compare);
      matches.pop_back();
    }

    index += 1;
  }

  // Sort from worst to best.
  std::sort_heap(matches.begin(), matches.end(), compare);
  // And reverse.
  std::reverse(matches.begin(), matches.end());
}

void findMatchesUsingClassifiers(const std::deque<Classifier>& classifiers,
                                 const std::deque<Descriptor>& points,
                                 std::deque<DirectedMatchList>& matches,
                                 int max_num_matches) {
  matches.clear();

  std::deque<Classifier>::const_iterator classifier;
  for (classifier = classifiers.begin();
       classifier != classifiers.end();
       ++classifier) {
    // Find matches for this classifier.
    DirectedMatchList classifier_matches;
    findMatchesUsingClassifier(*classifier, points, classifier_matches,
        max_num_matches);

    // Swap into the full list.
    matches.push_back(DirectedMatchList());
    matches.back().swap(classifier_matches);
  }
}

////////////////////////////////////////////////////////////////////////////////

namespace {

// A list of single matches.
typedef std::vector<cv::DMatch> RawMatchList;

// Converts from raw cv::Matches to DirectedMatches.
DirectedMatch convertMatch(const cv::DMatch& match) {
  return DirectedMatch(match.trainIdx, match.distance);
}

void convertMatchList(const RawMatchList& raw, DirectedMatchList& matches) {
  matches.clear();
  std::transform(raw.begin(), raw.end(), std::back_inserter(matches),
      convertMatch);
}

void convertMatchLists(const std::vector<RawMatchList>& raw,
                       std::deque<DirectedMatchList>& directed) {
  directed.assign(raw.size(), DirectedMatchList());

  std::vector<RawMatchList>::const_iterator raw_matches = raw.begin();
  std::deque<DirectedMatchList>::iterator directed_matches = directed.begin();

  while (raw_matches != raw.end()) {
    convertMatchList(*raw_matches, *directed_matches);

    ++raw_matches;
    ++directed_matches;
  }
}


// Iteratively finds matches within a relative radius of the best match.
// Iterative in case of approximate techniques finding better "best" matches.
//
// Parameters:
// query -- Single-row matrix
void relativeRadiusMatchIterative(const cv::Mat& query,
                                  cv::DescriptorMatcher& matcher,
                                  RawMatchList& matches,
                                  double nearest,
                                  double max_relative_distance) {
  CHECK(query.rows == 1);
  // Even though we're only matching one element, deque of vectors returned.
  std::vector<RawMatchList> singleton;

  bool changed = true;

  while (changed) {
    // Compute new radius.
    double radius = nearest / max_relative_distance;
    // Find matches within radius.
    matcher.radiusMatch(query, singleton, radius);
    matches.swap(singleton.front());

    // Buffer previous distance.
    double prev_nearest = nearest;
    // Find new nearest distance.
    nearest = matches.front().distance;

    // Was the new nearest-distance different to the old one?
    changed = (nearest != prev_nearest);
  }
}

void relativeRadiusMatch(const cv::Mat& query,
                         cv::DescriptorMatcher& matcher,
                         std::vector<RawMatchList>& match_lists,
                         double max_relative_distance) {
  // Initialize output.
  match_lists.assign(query.rows, RawMatchList());

  // Find the single best match for each.
  RawMatchList best;
  matcher.match(query, best);

  std::vector<RawMatchList>::iterator matches = match_lists.begin();

  // Now search for each query descriptor using a different search radius.
  for (int i = 0; i < query.rows; i += 1) {
    // Compute search radius using best match.
    double nearest = best[i].distance;

    // Iteratively find nearest.
    // In the case of brute force, will only iterate once.
    relativeRadiusMatchIterative(query.row(i), matcher, *matches, nearest,
        max_relative_distance);

    ++matches;
  }
}

// Removes matches which are not within the max relative distance.
void filterMatches(const DirectedMatchList& matches,
                   DirectedMatchList& subset,
                   double max_relative_distance) {
  // Matches are ordered by distance.
  // Find the point at which they exceed the distance.
  double nearest = matches.front().distance;
  double radius = nearest / max_relative_distance;

  DirectedMatchList::const_iterator last = matches.begin();
  while (last != matches.end() && last->distance <= radius) {
    ++last;
  }

  subset.clear();
  std::copy(matches.begin(), last, std::back_inserter(subset));
}

void filterMatchLists(const std::deque<DirectedMatchList>& input,
                      std::deque<DirectedMatchList>& output,
                      double max_relative_distance) {
  output.assign(input.size(), DirectedMatchList());

  std::deque<DirectedMatchList>::const_iterator input_matches = input.begin();
  std::deque<DirectedMatchList>::iterator output_matches = output.begin();

  while (input_matches != input.begin()) {
    filterMatches(*input_matches, *output_matches, max_relative_distance);

    ++input_matches;
    ++output_matches;
  }
}

// If max_num_matches is greater than zero, the number of matches will be
// limited. If max_relative_distance is greater than zero, the distance of the
// matches from the best match will be limited.
//
// If both constraints are active, the fixed number will be retrieved and
// results that are too far away will be removed.
void findMatchesUsingEuclideanDistance(const cv::Mat& query,
                                       const cv::Mat& train,
                                       std::deque<DirectedMatchList>& matches,
                                       int max_num_matches,
                                       double max_relative_distance,
                                       bool use_flann) {
  // Construct matching object.
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (use_flann) {
    matcher = cv::DescriptorMatcher::create("FlannBased");
  } else {
    matcher = cv::DescriptorMatcher::create("BruteForce");
  }

  // Put one element in a singleton list to train the matcher.
  std::vector<cv::Mat> singleton;
  singleton.push_back(train);
  matcher->add(singleton);

  if (max_num_matches > 0) {
    // Take top few matches.
    std::vector<RawMatchList> raw;
    matcher->knnMatch(query, raw, max_num_matches);

    // Convert from cv::DMatch to our match.
    convertMatchLists(raw, matches);

    if (max_relative_distance > 0) {
      // Remove any results that are too far away.
      std::deque<DirectedMatchList> filtered;
      filterMatchLists(matches, filtered, max_relative_distance);
      matches.swap(filtered);
    }
  } else {
    // No maximum number of matches specified.
    CHECK(max_relative_distance > 0) << "No limit on number of matches";

    // Retrieve based on relative distance.
    std::vector<RawMatchList> raw;
    relativeRadiusMatch(query, *matcher, raw, max_relative_distance);

    // Convert from cv::DMatch to our match.
    convertMatchLists(raw, matches);
  }
}

}

void findMatchesInBothDirectionsUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::deque<DirectedMatchList>& forward,
    std::deque<DirectedMatchList>& reverse,
    int max_num_matches,
    double max_relative_distance,
    bool use_flann) {
  // Copy descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(points1, mat1);
  listToMatrix(points2, mat2);

  // Match forwards and backwards.
  findMatchesUsingEuclideanDistance(mat1, mat2, forward, max_num_matches,
      max_relative_distance, use_flann);
  findMatchesUsingEuclideanDistance(mat2, mat1, reverse, max_num_matches,
      max_relative_distance, use_flann);
}

////////////////////////////////////////////////////////////////////////////////

namespace {

void addMatchesToSet(const DirectedMatchList& directed,
                     int index,
                     bool forward,
                     std::set<MatchResult>& undirected) {
  DirectedMatchList::const_iterator d;

  for (d = directed.begin(); d != directed.end(); ++d) {
    int index1 = index;
    int index2 = d->index;

    if (!forward) {
      std::swap(index1, index2);
    }

    MatchResult u(index1, index2, d->distance);
    undirected.insert(u);
  }
}

void addAllMatchesToSet(const std::deque<DirectedMatchList>& directed,
                        bool forward,
                        std::set<MatchResult>& undirected) {
  std::deque<DirectedMatchList>::const_iterator d;
  int index = 0;

  for (d = directed.begin(); d != directed.end(); ++d) {
    addMatchesToSet(*d, index, forward, undirected);
    index += 1;
  }
}

}

void intersectionOfMatchLists(
    const std::deque<DirectedMatchList>& forward_matches,
    const std::deque<DirectedMatchList>& reverse_matches,
    std::vector<MatchResult>& matches) {
  // Construct a set of undirected matches from both sets of directed matches.
  std::set<MatchResult> forward_set;
  std::set<MatchResult> reverse_set;
  addAllMatchesToSet(forward_matches, true, forward_set);
  addAllMatchesToSet(reverse_matches, false, reverse_set);

  matches.clear();
  std::set_intersection(forward_set.begin(), forward_set.end(),
      reverse_set.begin(), reverse_set.end(), std::back_inserter(matches));
}

void unionOfMatchLists(
    const std::deque<DirectedMatchList>& forward_matches,
    const std::deque<DirectedMatchList>& reverse_matches,
    std::vector<MatchResult>& matches) {
  // Construct a set of undirected matches from both sets of directed matches.
  std::set<MatchResult> forward_set;
  std::set<MatchResult> reverse_set;
  addAllMatchesToSet(forward_matches, true, forward_set);
  addAllMatchesToSet(reverse_matches, false, reverse_set);

  matches.clear();
  std::set_union(forward_set.begin(), forward_set.end(), reverse_set.begin(),
      reverse_set.end(), std::back_inserter(matches));
}
