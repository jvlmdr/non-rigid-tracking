#include <iostream>
#include <string>
#include <sstream>
#include <stack>
#include <map>
#include <set>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include "match_result.hpp"
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "feature_index.hpp"
#include "match_graph.hpp"
#include "feature_sets.hpp"

#include "read_lines.hpp"
#include "multiview_track_list_reader.hpp"
#include "default_reader.hpp"
#include "iterator_reader.hpp"
#include "match_result_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "iterator_writer.hpp"
#include "default_writer.hpp"

typedef boost::property_map<MatchGraph, boost::edge_weight_t>::const_type
        EdgeWeightMap;

typedef MatchGraph::vertex_descriptor Vertex;
typedef std::map<FeatureIndex, int> VertexLookup;
typedef std::set<Vertex> VertexSet;

////////////////////////////////////////////////////////////////////////////////

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Finds tracks by agglomerative clustering." << std::endl;
  usage << std::endl;
  usage << argv[0] << " initial-tracks matches-format view-names num-frames "
      "tracks" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

std::string makeMatchFilename(const std::string& format,
                              const std::string& view1,
                              const std::string& view2,
                              int time1,
                              int time2) {
  return boost::str(
      boost::format(format) % view1 % view2 % (time1 + 1) % (time2 + 1));
}

Vertex findOrInsert(MatchGraph& graph,
                    VertexLookup& vertices,
                    const FeatureIndex& feature) {
  Vertex vertex;

  // Try to find feature in lookup map.
  VertexLookup::const_iterator mapping = vertices.find(feature);

  if (mapping != vertices.end()) {
    // Found the vertex, it already exists.
    vertex = mapping->second;
  } else {
    // Did not find it, create an entry.
    vertex = boost::add_vertex(graph);
    graph[vertex] = feature;
    // Create an entry in the lookup table too.
    vertices[feature] = vertex;
  }

  return vertex;
}

void loadAllMatches(const std::string& matches_format,
                    const std::vector<std::string>& views,
                    int num_frames,
                    MatchGraph& graph,
                    VertexLookup& vertices) {
  vertices.clear();

  int num_views = views.size();
  int n = num_views * num_frames;
  int num_matches = 0;

  for (int i1 = 0; i1 < n; i1 += 1) {
    // Match all unique pairs.
    for (int i2 = i1 + 1; i2 < n; i2 += 1) {
      // Extract view and time indices.
      int t1 = i1 % num_frames;
      int t2 = i2 % num_frames;
      int v1 = i1 / num_frames;
      int v2 = i2 / num_frames;

      // Construct filename.
      std::string matches_file = makeMatchFilename(matches_format, views[v1],
          views[v2], t1, t2);

      // Load matches from file.
      std::vector<MatchResult> match_list;
      MatchResultReader reader;
      bool ok = loadList(matches_file, match_list, reader);
      CHECK(ok) << "Could not load matches";
      DLOG(INFO) << "Loaded " << match_list.size() << " matches for (" <<
          views[v1] << ", " << t1 << "), (" << views[v2] << ", " << t2 << ")";
      num_matches += match_list.size();

      // Add matches to map.
      Frame frame1(v1, t1);
      Frame frame2(v2, t2);

      std::vector<MatchResult>::const_iterator match;
      for (match = match_list.begin(); match != match_list.end(); ++match) {
        FeatureIndex feature1(frame1, match->index1);
        FeatureIndex feature2(frame2, match->index2);

        // Find existing vertex for feature, or insert one.
        Vertex vertex1 = findOrInsert(graph, vertices, feature1);
        Vertex vertex2 = findOrInsert(graph, vertices, feature2);

        // Add vertex to graph. Set edge weight to distance.
        MatchGraphEdgeProperty edge(match->distance);
        boost::add_edge(vertex1, vertex2, edge, graph);
      }
    }
  }
}

class CompareEdges {
  public:
    CompareEdges(const MatchGraph& graph) : weights_() {
      weights_ = boost::get(boost::edge_weight_t(), graph);
    }

    bool operator()(const MatchGraph::edge_descriptor& lhs,
                    const MatchGraph::edge_descriptor& rhs) {
      return weights_[lhs] > weights_[rhs];
    }

  private:
    EdgeWeightMap weights_;
};

void subsetToTrack(const MatchGraph& graph,
                   const std::map<Frame, int>& set,
                   MultiviewTrack<int>& track,
                   int num_views) {
  track = MultiviewTrack<int>(num_views);

  std::map<Frame, int>::const_iterator feature;
  for (feature = set.begin(); feature != set.end(); ++feature) {
    const FeatureIndex& index = graph[feature->second];
    track.view(index.view)[index.time] = index.id;
  }
}

// The set property is not important.
// TODO: Is there a way to avoid templating this code?
template<class T>
void subsetsToTracks(const MatchGraph& graph,
                     const FeatureSets<T>& sets,
                     MultiviewTrackList<int>& tracks,
                     int num_views) {
  tracks = MultiviewTrackList<int>(num_views);

  typename FeatureSets<T>::const_iterator set;
  for (set = sets.begin(); set != sets.end(); ++set) {
    MultiviewTrack<int> track;
    subsetToTrack(graph, set->second.elements, track, num_views);

    tracks.push_back(MultiviewTrack<int>());
    tracks.back().swap(track);
  }
}

struct Edge {
  Vertex source;
  Vertex target;
  double weight;

  Edge(Vertex source, Vertex target, double weight);

  bool operator<(const Edge& other) const;

  static Edge make(MatchGraph::edge_descriptor edge, const MatchGraph& graph);
};

Edge::Edge(Vertex source, Vertex target, double weight)
    : source(source), target(target), weight(weight) {}

bool Edge::operator<(const Edge& other) const {
  return weight > other.weight;
}

Edge Edge::make(MatchGraph::edge_descriptor edge, const MatchGraph& graph) {
  EdgeWeightMap weights = boost::get(boost::edge_weight_t(), graph);

  Vertex source = boost::source(edge, graph);
  Vertex target = boost::target(edge, graph);
  double weight = weights[edge];

  return Edge(source, target, weight);
}

// The properties of joining two sets.
struct JoinProperties {
  double residual;
  double uncertainty;
};

// The properties associated with a set.
struct SetProperties {
  // For each set, store a compatibility with connected sets.
  std::map<int, JoinProperties> affinity;
};

int main(int argc, char** argv) {
  init(argc, argv);

  std::string initial_tracks_file = argv[1];
  std::string matches_format = argv[2];
  std::string view_names_file = argv[3];
  int num_frames = boost::lexical_cast<int>(argv[4]);
  std::string tracks_file = argv[5];

  bool ok;

  // Load view names.
  std::vector<std::string> views;
  ok = readLines(view_names_file, views);
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load initial tracks.
  MultiviewTrackList<int> initial_tracks;
  DefaultReader<int> int_reader;
  ok = loadMultiviewTrackList(initial_tracks_file, initial_tracks, int_reader);
  CHECK(ok) << "Could not load initial tracks";

  // Load all matches into graph.
  MatchGraph graph;
  VertexLookup lookup;
  loadAllMatches(matches_format, views, num_frames, graph, lookup);
  int num_vertices = boost::num_vertices(graph);
  int num_edges = boost::num_edges(graph);
  LOG(INFO) << "Loaded " << num_edges << " matches between " << num_vertices <<
      " features";

  // Use a disjoint-sets data structure for union-find operations.
  FeatureSets<SetProperties> sets;
  
  std::vector<Frame> frames;
  {
    MatchGraph::vertex_iterator vertex;
    MatchGraph::vertex_iterator end;
    boost::tie(vertex, end) = boost::vertices(graph);

    for(; vertex != end; ++vertex) {
      const FeatureIndex& feature = graph[*vertex];
      frames.push_back(Frame(feature.view, feature.time));
    }
  }

  sets.init(frames, initial_tracks, lookup);

  LOG(INFO) << "Initially " << num_vertices << " vertices amongst " <<
      sets.count() << " sets";

  LOG(INFO) << "Building edge list";
  std::vector<Edge> edges;
  {
    MatchGraph::edge_iterator begin;
    MatchGraph::edge_iterator end;
    boost::tie(begin, end) = boost::edges(graph);
    std::transform(begin, end, std::back_inserter(edges),
        boost::bind(Edge::make, _1, graph));
  }

  LOG(INFO) << "Building heap";
  std::make_heap(edges.begin(), edges.end());

  while (!edges.empty()) {
    // Pull the first edge off the heap.
    Edge edge = edges.front();
    std::pop_heap(edges.begin(), edges.end());
    edges.pop_back();

    // Skip if elements are already in the same set.
    if (!sets.together(edge.source, edge.target)) {
      DLOG(INFO) << "(" << edge.source << ", " << edge.target << ") => " <<
          edge.weight;

      // Only merge if sets are compatible.
      if (sets.compatible(edge.source, edge.target)) {
        sets.join(edge.source, edge.target);
        DLOG(INFO) << "Merged: " << sets.count() << " sets";

        // TODO: Compute set properties...
      } else {
        DLOG(INFO) << "Inconsistent";
      }
    }
  }

  LOG(INFO) << "Split " << num_vertices << " vertices into " << sets.count() <<
      " sets";

  // Convert each consistent subgraph to a multi-view track of indices.
  MultiviewTrackList<int> tracks;
  subsetsToTracks(graph, sets, tracks, num_views);

  // Save.
  DefaultWriter<int> writer;
  ok = saveMultiviewTrackList(tracks_file, tracks, writer);
  CHECK(ok) << "Could not save tracks file";
  
  return 0;
}
