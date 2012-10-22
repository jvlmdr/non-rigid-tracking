#ifndef MATCH_GRAPH_HPP_
#define MATCH_GRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include "feature_index.hpp"

typedef boost::property<boost::edge_weight_t, double>
        MatchGraphEdgeProperty;

typedef boost::adjacency_list<boost::vecS,
                              boost::vecS,
                              boost::undirectedS,
                              FeatureIndex,
                              MatchGraphEdgeProperty>
        MatchGraph;

#endif
