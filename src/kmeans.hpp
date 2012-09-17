#ifndef KMEANS_HPP_
#define KMEANS_HPP_

#include <vector>
#include <deque>
#include <boost/random/mersenne_twister.hpp>

// A k-means points must be able to expose a vector representation.
class KMeansPoint {
  public:
    virtual ~KMeansPoint() {}
    virtual const std::vector<double>& vector() const = 0;
};

// Runs k-means given points and initial cluster centers.
//
// Parameters:
// points -- Points to cluster.
//   Contain at least one element.
//   Every point has the same (non-zero) dimension.
// centers -- Initial cluster centers. Same assumptions as above.
//   Final cluster centers will be written out here.
// labels -- Cluster membership of each point will be written out here.
//
// Terminates at convergence (when nothing changes).
void kMeans(const std::vector<const KMeansPoint*>& points,
            std::deque<std::vector<double> >& centers,
            std::vector<int>& labels);

// Runs k-means with randomized cluster centers.
void randomKMeans(const std::vector<const KMeansPoint*>& points,
                  int k,
                  std::deque<std::vector<double> >& centers,
                  std::vector<int>& labels,
                  boost::random::mt19937& generator);

// Finds the nearest cluster center for each point. Parameters as above.
//
// Returns the number of points whose labels were modified.
// If the input labels were empty, all labels are considered modified.
int assignEachPointToCluster(const std::vector<const KMeansPoint*>& points,
                             const std::deque<std::vector<double> >& centers,
                             std::vector<int>& labels);

void removeEmptyClusters(std::deque<std::vector<double> >& centers,
                         std::vector<int>& labels);

void updateClusterPositions(const std::vector<const KMeansPoint*>& points,
                            std::deque<std::vector<double> >& centers,
                            const std::vector<int>& labels,
                            int k);

// Picks random points as cluster centers.
void randomClusterCenters(const std::vector<const KMeansPoint*>& points,
                          int k,
                          std::deque<std::vector<double> >& centers,
                          boost::random::mt19937& generator);

#endif
