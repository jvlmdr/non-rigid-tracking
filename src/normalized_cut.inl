#include <opencv2/core/core.hpp>
#include <map>
#include <algorithm>
#include <numeric>
#include <glog/logging.h>
#include "sparse_mat.hpp"
#include <boost/graph/adjacency_list.hpp>

bool smallestEigenvector(const cv::SparseMat& A,
                         int k,
                         std::vector<double>* x,
                         double* lambda,
                         int max_iter);

// Graph must satify VertexListGraph and EdgeListGraph. 
template<class Graph>
void graphEdgesToSparseMatrix(const Graph& graph, cv::SparseMat& A) {
  int n = boost::num_vertices(graph);

  const int ndims = 2;
  const int dims[ndims] = { n, n };
  A.create(ndims, dims, cv::DataType<double>::type);

  typedef typename Graph::edge_iterator EdgeIterator;
  std::pair<EdgeIterator, EdgeIterator> edges;
  edges = boost::edges(graph);

  // Get edge weight property map.
  typedef typename boost::property_map<Graph, boost::edge_weight_t>::const_type
          WeightMap;
  WeightMap weights = boost::get(boost::edge_weight_t(), graph);

  EdgeIterator edge;
  for (edge = edges.first; edge != edges.second; ++edge) {
    int i = boost::source(*edge, graph);
    int j = boost::target(*edge, graph);
    double w = boost::get(weights, *edge);

    CHECK(i < n);
    CHECK(j < n);

    A.ref<double>(i, j) = w;
    A.ref<double>(j, i) = w;
  }
}

template<class Graph>
double quantizeUpToPositiveScale(const std::multimap<double, int>& map,
                                 std::vector<int>& labels,
                                 const Graph& graph,
                                 const std::vector<double>& degrees) {
  int n = boost::num_vertices(graph);
  double volume = std::accumulate(degrees.begin(), degrees.end(), 0.);

  labels.clear();
  double min_ncut = 0;

  // Tentative labels. Initially every point in set A.
  std::vector<int> y(n, 0);

  // Incrementally update volumes and cut.
  double volume_A = volume;
  double volume_B = 0;
  double cut = 0;

  std::multimap<double, int>::const_iterator iter = map.begin();;

  // Get edge weight property map.
  typedef typename boost::property_map<Graph, boost::edge_weight_t>::const_type
          WeightMap;
  WeightMap weights = boost::get(boost::edge_weight_t(), graph);

  while (iter != map.end()) {
    // Get value of current element.
    double value = iter->first;

    // Add all non-positive elements in the first iteration.
    // Add all elements which have identical value.
    while (iter != map.end() && (iter->first <= 0 || iter->first == value)) {
      int u = iter->second;

      // Iterate through adjacent vertices.
      typedef typename Graph::out_edge_iterator EdgeIterator;
      std::pair<EdgeIterator, EdgeIterator> edges;
      edges = boost::out_edges(u, graph);

      EdgeIterator edge;
      for (edge = edges.first; edge != edges.second; ++edge) {
        // Edge is to vertex v with weight w.
        int v = boost::target(*edge, graph);
        double w = boost::get(weights, *edge);

        if (y[v] == 0) {
          // Neighbor remains in old set. Edge is cut.
          cut += w;
        } else {
          // Neighbor is in new set. Edge is restored.
          cut -= w;
        }
      }

      // Remove from set A.
      volume_A -= degrees[u];
      // Add to set B.
      volume_B += degrees[u];

      // Move this vertex to B.
      y[u] = 1;

      ++iter;
    }

    if (iter != map.end()) {
      double ncut = (1. / volume_A + 1. / volume_B) * cut;

      // If this is the best so far...
      if (labels.empty() || ncut < min_ncut) {
        labels = y;
        min_ncut = ncut;
      }
    }
  }

  CHECK_EQ(volume_A, 0);
  CHECK_EQ(volume_B, volume);

  return min_ncut;
}

// Returns a discrete solution given the solution to the continuous relaxation.
//
// The continuous solution is only up to scale.
// Searches the scale parameter for the optimal solution w.r.t. objective.
template<class Graph>
void quantizeUpToScale(const std::vector<double>& x,
                       std::vector<int>& labels,
                       const Graph& graph,
                       const std::vector<double>& degrees) {
  int n = x.size();

  std::vector<int> positive_labels;
  double positive_cut;
  {
    // Construct a value to index map.
    std::multimap<double, int> map;
    for (int i = 0; i < n; i += 1) {
      map.insert(std::make_pair(x[i], i));
    }

    positive_cut = quantizeUpToPositiveScale(map, positive_labels, graph,
        degrees);
  }

  std::vector<int> negative_labels;
  double negative_cut;
  {
    // Construct a value to index map.
    std::multimap<double, int> map;
    for (int i = 0; i < n; i += 1) {
      map.insert(std::make_pair(x[i], i));
    }

    negative_cut = quantizeUpToPositiveScale(map, negative_labels, graph,
        degrees);
  }

  if (positive_cut <= negative_cut) {
    labels.swap(positive_labels);
  } else {
    labels.swap(negative_labels);
  }
}

template<class Graph>
bool normalizedCut(const Graph& graph,
                   std::vector<int>& labels,
                   int max_iter) {
  int n = boost::num_vertices(graph);

  // Populate sparse adjacency matrix.
  cv::SparseMat A;
  graphEdgesToSparseMatrix(graph, A);

  // Build nx1 matrix of the sums of each row of adjacency matrix.
  cv::Mat degrees;
  multiply(A, cv::Mat_<double>::ones(n, 1), degrees);

  // Compute d^0.5.
  cv::Mat root_degrees;
  cv::sqrt(degrees, root_degrees);

  // Compute 1 / d^0.5.
  cv::Mat inv_root_degrees;
  cv::divide(1, root_degrees, inv_root_degrees);

  // Build matrix from adjacency matrix.
  cv::SparseMat D;
  diag(degrees, D);
  // L = D - A
  cv::SparseMat L;
  A.convertTo(L, cv::DataType<double>::type, -1);
  addTo(D, L);
  // L = D^{-0.5} (D - A) D^{-0.5}.
  leftMultiplyByDiag(L, inv_root_degrees);
  rightMultiplyByDiag(L, inv_root_degrees);

  LOG(INFO) << "Solving " << n << "x" << n << " eigensystem";
  double lambda;
  std::vector<double> x;
  bool ok = smallestEigenvector(L, 1, &x, &lambda, max_iter);

  if (!ok) {
    LOG(WARNING) << "Reached iteration limit";
    return false;
  }

  quantizeUpToScale(x, labels, graph, degrees);

  int n1 = 0;
  int n2 = 0;

  // Split solutions using sign.
  for (int i = 0; i < n; i += 1) {
    if (labels[i] == 0) {
      n1 += 1;
    } else {
      n2 += 1;
    }
  }
  LOG(INFO) << n << " => {" << n1 << ", " << n2 << "}";

  return true;
}
