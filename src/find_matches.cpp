#include "find_matches.hpp"
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "find_matches_util.hpp"

QueryResult::QueryResult() : index(-1), distance(-1) {}

QueryResult::QueryResult(int index, double distance)
    : index(index), distance(distance) {}

////////////////////////////////////////////////////////////////////////////////

void convertQueryResultListsToMatches(
    const std::deque<QueryResultList>& queries,
    std::vector<MatchResult>& matches,
    bool forward) {
  matches.clear();

  std::deque<QueryResultList>::const_iterator list;
  int index = 0;

  for (list = queries.begin(); list != queries.end(); ++list) {
    QueryResultList::const_iterator query;

    for (query = list->begin(); query != list->end(); ++query) {
      int index1 = index;
      int index2 = query->index;

      if (!forward) {
        std::swap(index1, index2);
      }

      matches.push_back(MatchResult(index1, index2, query->distance));
    }

    index += 1;
  }
}

////////////////////////////////////////////////////////////////////////////////

class CompareQueryResults {
  public:
    CompareQueryResults(bool ascending);

    bool operator()(const QueryResult& lhs, const QueryResult& rhs) const;

  private:
    bool ascending_;
};

CompareQueryResults::CompareQueryResults(bool ascending)
    : ascending_(ascending) {}

bool CompareQueryResults::operator()(const QueryResult& lhs,
                                        const QueryResult& rhs) const {
  if (ascending_) {
    return lhs.distance < rhs.distance;
  } else {
    return lhs.distance > rhs.distance;
  }
}

void findMatchesUsingClassifier(const Classifier& classifier,
                                const std::deque<Descriptor>& points,
                                QueryResultList& matches,
                                bool use_max_num,
                                int max_num,
                                bool use_threshold,
                                double threshold) {
  if (use_max_num) {
    CHECK(max_num > 0);
  }

  matches.clear();

  // Use a heap to keep track of the worst (sort highest to lowest by distance).
  CompareQueryResults compare(true);

  std::deque<Descriptor>::const_iterator point;
  int index = 0;

  for (point = points.begin(); point != points.end(); ++point) {
    double distance = std::exp(-classifier.score(*point));

    bool close_enough;
    if (!use_threshold) {
      close_enough = true;
    } else {
      close_enough = (distance <= threshold);
    }

    if (close_enough) {
      // If we haven't reached the maximum number of matches or this match is
      // better than the worst, add it to the heap.
      bool reached_limit;
      if (!use_max_num) {
        reached_limit = false;
      } else {
        reached_limit = (int(matches.size()) >= max_num);
      }

      bool add;
      if (!reached_limit) {
        add = true;
      } else {
        add = (distance < matches.front().distance);
      }

      if (add) {
        matches.push_back(QueryResult(index, distance));
        std::push_heap(matches.begin(), matches.end(), compare);

        // If there are now too many elements, remove from the heap.
        bool too_many;
        if (!use_max_num) {
          too_many = false;
        } else {
          too_many = (int(matches.size()) > max_num);
        }

        if (too_many) {
          std::pop_heap(matches.begin(), matches.end(), compare);
          matches.pop_back();
        }
      }
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
                                 std::deque<QueryResultList>& matches,
                                 bool use_max_num,
                                 int max_num,
                                 bool use_threshold,
                                 double threshold) {
  matches.clear();

  std::deque<Classifier>::const_iterator classifier;
  for (classifier = classifiers.begin();
       classifier != classifiers.end();
       ++classifier) {
    // Find matches for this classifier.
    QueryResultList classifier_matches;
    findMatchesUsingClassifier(*classifier, points, classifier_matches,
        use_max_num, max_num, use_threshold, threshold);

    // Swap into the full list.
    matches.push_back(QueryResultList());
    matches.back().swap(classifier_matches);
  }
}

////////////////////////////////////////////////////////////////////////////////

namespace {

// A list of single matches.
typedef std::vector<cv::DMatch> RawMatchList;

// Converts from raw cv::Matches to query results.
QueryResult convertMatch(const cv::DMatch& match) {
  return QueryResult(match.trainIdx, match.distance);
}

void convertMatchList(const RawMatchList& raw, QueryResultList& matches) {
  matches.clear();
  std::transform(raw.begin(), raw.end(), std::back_inserter(matches),
      convertMatch);
}

void convertMatchLists(const std::vector<RawMatchList>& raw,
                       std::deque<QueryResultList>& directed) {
  directed.assign(raw.size(), QueryResultList());

  std::vector<RawMatchList>::const_iterator raw_matches = raw.begin();
  std::deque<QueryResultList>::iterator directed_matches = directed.begin();

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
void filterMatches(const QueryResultList& matches,
                   QueryResultList& subset,
                   double max_relative_distance) {
  // Matches are ordered by distance.
  // Find the point at which they exceed the distance.
  double nearest = matches.front().distance;
  double radius = nearest / max_relative_distance;

  QueryResultList::const_iterator last = matches.begin();
  while (last != matches.end() && last->distance <= radius) {
    ++last;
  }

  subset.clear();
  std::copy(matches.begin(), last, std::back_inserter(subset));
}

void filterMatchLists(const std::deque<QueryResultList>& input,
                      std::deque<QueryResultList>& output,
                      double max_relative_distance) {
  output.assign(input.size(), QueryResultList());

  std::deque<QueryResultList>::const_iterator input_matches = input.begin();
  std::deque<QueryResultList>::iterator output_matches = output.begin();

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
void matchMatrixRows(const cv::Mat& query,
                     const cv::Mat& train,
                     std::deque<QueryResultList>& matches,
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
      std::deque<QueryResultList> filtered;
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

void findMatchesUsingEuclideanDistance(const std::deque<Descriptor>& points1,
                                       const std::deque<Descriptor>& points2,
                                       std::deque<QueryResultList>& matches,
                                       int max_num_matches,
                                       double max_relative_distance,
                                       bool use_flann) {
  // Copy descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(points1, mat1);
  listToMatrix(points2, mat2);

  // Match forwards.
  matchMatrixRows(mat1, mat2, matches, max_num_matches, max_relative_distance,
      use_flann);
}

void findMatchesInBothDirectionsUsingEuclideanDistance(
    const std::deque<Descriptor>& points1,
    const std::deque<Descriptor>& points2,
    std::deque<QueryResultList>& forward,
    std::deque<QueryResultList>& reverse,
    int max_num_matches,
    double max_relative_distance,
    bool use_flann) {
  // Copy descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(points1, mat1);
  listToMatrix(points2, mat2);

  // Match forwards and backwards.
  matchMatrixRows(mat1, mat2, forward, max_num_matches, max_relative_distance,
      use_flann);
  matchMatrixRows(mat2, mat1, reverse, max_num_matches, max_relative_distance,
      use_flann);
}
