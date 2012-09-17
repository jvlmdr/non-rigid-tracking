#include "kmeans.hpp"
#include <boost/random/uniform_int_distribution.hpp>
#include <set>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include "util.hpp"

// "m << n" means "MUCH_LARGER * m < n"
// Expected number of trials to draw m unique elements from n versus O(n).
const int MUCH_LARGER = 2;

namespace{

typedef std::vector<double> Vector;

class Result {
  public:
    inline int index() const { return index_; }
    inline double value() const { return value_; }
    inline bool empty() const { return index_ < 0; }

    Result() : index_(-1), value_(0) {}

    Result(const Result& other)
        : empty_(other.empty_), index_(other.index_), value_(other.value_) {}

    Result(int index, double value)
        : empty_(false), index_(index), value_(value) {}

  private:
    bool empty_;
    int index_;
    double value_;
};

// Assumes x and y are the same dimension!
double squaredDistance(const Vector& x,
                       const Vector& y) {
  CHECK(x.size() == y.size());

  cv::Mat e;
  cv::subtract(x, y, e);

  return e.dot(e);
}

Result findNearest(const Vector& query,
                   const std::deque<Vector>& points) {
  Result min;
  int num_min = 0;

  std::deque<Vector>::const_iterator point;
  int i = 0;

  for (point = points.begin(); point != points.end(); ++point) {
    // Compute distance.
    double distance = squaredDistance(query, *point);

    if (min.empty() || distance < min.value()) {
      // Replace nearest.
      min = Result(i, distance);
      num_min = 1;
    } else if (distance == min.value()) {
      // Found an identical minimum.
      num_min += 1;
    }

    i += 1;
  }

  if (num_min > 1) {
    DLOG(WARNING) << "Exact tie between " << num_min << " clusters";
  }

  CHECK(min.index() >= 0);
  CHECK(min.index() < int(points.size()));

  return min;
}

}

void kMeans(const std::vector<const KMeansPoint*>& points,
            std::deque<Vector>& centers,
            std::vector<int>& labels) {
  bool converged = false;
  int iter = 0;

  assignEachPointToCluster(points, centers, labels);

  while (!converged) {
    updateClusterPositions(points, centers, labels, centers.size());

    int num_changed = assignEachPointToCluster(points, centers, labels);
    converged = (num_changed == 0);

    DLOG(INFO) << "Iteration " << iter << ": " << num_changed << "/" <<
        points.size() << " points switched cluster";

    if (!converged) {
      removeEmptyClusters(centers, labels);
    }

    iter += 1;
  }
}

void randomKMeans(const std::vector<const KMeansPoint*>& points,
                  int k,
                  std::deque<Vector>& centers,
                  std::vector<int>& labels,
                  boost::random::mt19937& generator) {
  randomClusterCenters(points, k, centers, generator);
  kMeans(points, centers, labels);
}

int assignEachPointToCluster(const std::vector<const KMeansPoint*>& points,
                             const std::deque<Vector>& centers,
                             std::vector<int>& labels) {
  int n = points.size();

  // Check whether labels had valid initialization.
  if (int(labels.size()) != n) {
    // If not, initialize all labels to -1.
    labels.assign(n, -1);
  }

  // Update label of each point.
  std::vector<const KMeansPoint*>::const_iterator point = points.begin();
  std::vector<int>::iterator label = labels.begin();
  int num_changed = 0;

  while (point != points.end()) {
    Result nearest = findNearest((*point)->vector(), centers);

    if (*label != nearest.index()) {
      num_changed += 1;
    }

    *label = nearest.index();

    ++point;
    ++label;
  }

  return num_changed;
}

namespace {

struct Cluster {
  Vector center;
  std::vector<int> points;

  void swap(Cluster& other) {
    center.swap(other.center);
    points.swap(other.points);
  }
};

}

namespace std {

template<>
void swap(Cluster& lhs, Cluster& rhs) {
  lhs.swap(rhs);
}

}

void labelsToLists(const std::vector<int>& labels,
                   std::deque<Vector>& centers,
                   std::deque<Cluster>& clusters) {
  clusters.clear();

  // Move centers into lists.
  std::deque<Vector>::iterator center;
  for (center = centers.begin(); center != centers.end(); ++center) {
    clusters.push_back(Cluster());
    clusters.back().center.swap(*center);
  }

  // Build list of elements having each label.
  int i = 0;
  std::vector<int>::const_iterator label;
  for (label = labels.begin(); label != labels.end(); ++label) {
    clusters[*label].points.push_back(i);
    i += 1;
  }
}

void listsToLabels(std::deque<Cluster>& clusters,
                   std::vector<int>& labels,
                   std::deque<Vector>& centers,
                   int n) {
  labels.assign(n, -1);
  centers.clear();

  int i = 0;
  std::deque<Cluster>::iterator cluster;
  for (cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
    // Set every element of that list to the current label.
    const std::vector<int>& points = cluster->points;
    std::vector<int>::const_iterator point;
    for (point = points.begin(); point != points.end(); ++point) {
      labels[*point] = i;
    }

    // Move position of cluster.
    centers.push_back(Vector());
    centers.back().swap(cluster->center);

    i += 1;
  }
}

void removeEmptyLists(std::deque<Cluster>& lists) {
  std::deque<Cluster> non_empty;

  std::deque<Cluster>::iterator list;
  for (list = lists.begin(); list != lists.end(); ++list) {
    if (!list->points.empty()) {
      // Move to non-empty list.
      non_empty.push_back(Cluster());
      non_empty.back().swap(*list);
    }
  }

  lists.swap(non_empty);
}

void removeEmptyClusters(std::deque<Vector>& centers,
                         std::vector<int>& labels) {
  int n = labels.size();
  int k = centers.size();

  // Sort points by their label.
  std::deque<Cluster> lists;
  labelsToLists(labels, centers, lists);

  removeEmptyLists(lists);

  // Back to labels.
  listsToLabels(lists, labels, centers, n);

  if (int(centers.size()) != k) {
    LOG(INFO) << "Empty cluster: decreasing from " << k << " to " <<
        centers.size();
  }
}

void updateClusterPositions(const std::vector<const KMeansPoint*>& points,
                            std::deque<Vector>& centers,
                            const std::vector<int>& labels,
                            int k) {
  centers.assign(k, Vector());
  std::vector<int> counts(k, 0);

  // Calculate each point's contribution to its center.
  std::vector<const KMeansPoint*>::const_iterator point = points.begin();
  std::vector<int>::const_iterator label = labels.begin();
  CHECK(labels.size() == points.size());

  while (point != points.end()) {
    if (centers[*label].empty()) {
      // Initialize sum.
      centers[*label] = (*point)->vector();
    } else {
      // Add to sum.
      cv::add(centers[*label], (*point)->vector(), centers[*label]);
    }
    ++counts[*label];

    ++point;
    ++label;
  }

  // Take averages.
  std::deque<Vector>::iterator center = centers.begin();
  std::vector<int>::const_iterator count = counts.begin();

  while (center != centers.end()) {
    if (*count == 0) {
      // Remove cluster!
      centers.erase(center);
    } else {
      // Make matrix without copying data.
      cv::Mat c(*center, false);
      c /= static_cast<double>(*count);
    }

    ++center;
    ++count;
  }
}

////////////////////////////////////////////////////////////////////////////////

// Generates m unique random numbers in [0, n) by trial and error.
// Only use for m << n.
void uniqueRandomNumbersByTrialAndError(int m,
                                        int n,
                                        std::vector<int>& seq,
                                        boost::random::mt19937& generator) {
  seq.clear();
  std::set<int> used;

  boost::random::uniform_int_distribution<> dist(0, n - 1);

  while (int(seq.size()) < m) {
    int r = dist(generator);

    // Have we already used this number?
    if (used.find(r) == used.end()) {
      // Nope! Use it.
      seq.push_back(r);
      used.insert(r);
    }
  }
}

namespace {

class GenerateFunction {
  public:
    GenerateFunction(boost::random::mt19937& generator)
        : generator_(&generator) {}

    int operator()(int n) {
      boost::random::uniform_int_distribution<> dist(0, n - 1);
      return dist(*generator_);
    }

  private:
    boost::random::mt19937* generator_;
};

}

void uniqueRandomNumbersByFullShuffle(int m,
                                      int n,
                                      std::vector<int>& seq,
                                      boost::random::mt19937& generator) {
  // Generate numbers 0, ..., n-1.
  std::vector<int> range;
  for (int i = 0; i < n; i += 1) {
    range.push_back(i);
  }

  // Shuffle full range.
  GenerateFunction function(generator);
  std::random_shuffle(range.begin(), range.end(), function);

  // Copy first m elements to seq.
  seq.clear();
  std::vector<int>::const_iterator r = seq.begin();
  for (int i = 0; i < m; i += 1) {
    seq.push_back(*r);
    ++r;
  }
}

// Use only if m << n.
void uniqueRandomNumbers(int m,
                         int n,
                         std::vector<int>& seq,
                         boost::random::mt19937& generator) {
  if (MUCH_LARGER * m < n) {
    uniqueRandomNumbersByTrialAndError(m, n, seq, generator);
  } else {
    uniqueRandomNumbersByFullShuffle(m, n, seq, generator);
  }
}

////////////////////////////////////////////////////////////////////////////////

void uniqueRandomElementsByTrialAndError(
    int m,
    const std::vector<const KMeansPoint*>& points,
    std::deque<Vector>& subset,
    boost::random::mt19937& generator) {
  int n = points.size();

  subset.clear();
  std::set<int> tried;

  boost::random::uniform_int_distribution<> dist(0, n - 1);

  while (int(subset.size()) < m && int(tried.size()) < n) {
    int r = dist(generator);

    // Have we already tried this number already?
    if (tried.find(r) != tried.end()) {
      // Yep! Skip.
      continue;
    }
    // If not, then we have now.
    tried.insert(r);

    // Compare to all selected points so far.
    bool found = false;
    std::deque<Vector>::const_iterator prev = subset.begin();

    while (prev != subset.end() && !found) {
      found = (squaredDistance(points[r]->vector(), *prev) == 0);
      ++prev;
    }

    // Use the number.
    if (!found) {
      subset.push_back(points[r]->vector());
    }
  }

  if (int(subset.size()) < m) {
    LOG(WARNING) << "Could only find " << subset.size() <<
        " unique points amongst " << points.size();
  }
}

void uniqueElements(
    const std::vector<const KMeansPoint*>& points,
    std::vector<const KMeansPoint*>& unique) {
  unique.clear();

  std::vector<const KMeansPoint*>::const_iterator point;
  for (point = points.begin(); point != points.end(); ++point) {
    // Compare to all selected points so far.
    bool found = false;

    std::vector<const KMeansPoint*>::const_iterator prev = unique.begin();
    while (prev != unique.end() && !found) {
      found = (squaredDistance((*point)->vector(), (*prev)->vector()) == 0);
      ++prev;
    }

    // Use the number.
    if (!found) {
      unique.push_back(*point);
    }
  }
}

void uniqueRandomElementsByFullShuffle(
    int m,
    const std::vector<const KMeansPoint*>& points,
    std::deque<Vector>& subset,
    boost::random::mt19937& generator) {
  // Remove duplicates. O(n^2)
  std::vector<const KMeansPoint*> unique;
  uniqueElements(points, unique);

  // Do a full shuffle. O(n)
  GenerateFunction function(generator);
  std::random_shuffle(unique.begin(), unique.end(), function);

  // Take first m elements.
  subset.clear();
  std::vector<const KMeansPoint*>::iterator iter = unique.begin();
  while (int(subset.size()) < m && iter != unique.end()) {
    subset.push_back((*iter)->vector());
    ++iter;
  }
}

// Generates indices of m unique random elements.
// Only use for m << n.
//
// CAUTION: Sequence must contain m unique elements.
// If it does not, it will fail, possibly after a very long time.
void uniqueRandomElements(int m,
                          const std::vector<const KMeansPoint*>& points,
                          std::deque<Vector>& subset,
                          boost::random::mt19937& generator) {
  int n = points.size();

  if (MUCH_LARGER * m < n) {
    uniqueRandomElementsByTrialAndError(m, points, subset, generator);
  } else {
    uniqueRandomElementsByFullShuffle(m, points, subset, generator);
  }
}

////////////////////////////////////////////////////////////////////////////////

void randomClusterCenters(const std::vector<const KMeansPoint*>& points,
                          int k,
                          std::deque<Vector>& centers,
                          boost::random::mt19937& generator) {
  // Pick cluster centers.
  uniqueRandomElements(k, points, centers, generator);
}
