#ifndef NORMALIZED_CUT_HPP_
#define NORMALIZED_CUT_HPP_

#include <vector>

// Finds the minimum normalized cut.
//
// Graph must satisfy VertexListGraph, EdgeListGraph and IncidenceGraph.
template<class Graph>
bool normalizedCut(const Graph& graph,
                   std::vector<int>& labels,
                   int max_iter);

#include "normalized_cut.inl"

#endif
