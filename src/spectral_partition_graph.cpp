#include <iostream>
#include <string>
#include <sstream>
#include <stack>
#include <map>
#include <set>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/typeof/typeof.hpp>

#include "match.hpp"
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "feature_index.hpp"
#include "normalized_cut.hpp"
#include "sparse_mat.hpp"

#include "read_lines.hpp"
#include "iterator_reader.hpp"
#include "match_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "iterator_writer.hpp"
#include "default_writer.hpp"

const int MIN_GRAPH_SIZE = 2;
const int MAX_GRAPH_SIZE = 10000;

DEFINE_int32(max_iter, 1000, "Maximum number of iterations");

// boost::subgraph requires vertex_index_t and edge_index_t.
typedef boost::property<boost::vertex_index_t, int,
                        FeatureIndex>
        VertexProperty;
typedef boost::property<boost::edge_index_t, int,
        boost::property<boost::edge_weight_t, int> >
        EdgeProperty;
typedef boost::subgraph<
          boost::adjacency_list<boost::vecS,
                                boost::vecS,
                                boost::undirectedS,
                                VertexProperty,
                                EdgeProperty> >
        Graph;

typedef std::map<FeatureIndex, Graph::vertex_descriptor> VertexLookup;

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Reduces matches to consistent tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " matches-format view-names num-frames tracks" <<
      std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 5) {
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

Graph::vertex_descriptor findOrInsert(Graph& graph,
                                      VertexLookup& vertices,
                                      const FeatureIndex& feature) {
  Graph::vertex_descriptor vertex;

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
                    Graph& graph) {
  int num_views = views.size();
  int n = num_views * num_frames;
  int num_matches = 0;

  VertexLookup vertices;

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
      std::vector<Match> match_list;
      MatchReader reader;
      bool ok = loadList(matches_file, match_list, reader);
      CHECK(ok) << "Could not load matches";
      DLOG(INFO) << "Loaded " << match_list.size() << " matches for (" <<
          views[v1] << ", " << t1 << "), (" << views[v2] << ", " << t2 << ")";
      num_matches += match_list.size();

      // Add matches to map.
      ImageIndex frame1(v1, t1);
      ImageIndex frame2(v2, t2);

      for (std::vector<Match>::const_iterator match = match_list.begin();
           match != match_list.end();
           ++match) {
        FeatureIndex feature1(frame1, match->first);
        FeatureIndex feature2(frame2, match->second);

        // Find existing vertex for feature, or insert one.
        Graph::vertex_descriptor vertex1;
        Graph::vertex_descriptor vertex2;
        vertex1 = findOrInsert(graph, vertices, feature1);
        vertex2 = findOrInsert(graph, vertices, feature2);

        // Add vertex to graph. Set edge weight to 1.
        EdgeProperty edge(0, 1);
        boost::add_edge(vertex1, vertex2, edge, graph);
      }
    }
  }
}

void splitIntoComponents(Graph& graph, std::vector<Graph*>& subgraphs) {
  int num_vertices = boost::num_vertices(graph);

  // Extract connected components.
  std::vector<int> labels(num_vertices);
  int num_components = boost::connected_components(graph, &labels.front());

  subgraphs.clear();
  for (int i = 0; i < num_components; i += 1) {
    // Initialize subgraphs.
    subgraphs.push_back(&graph.create_subgraph());
  }

  // Add each vertex to one subgraph.
  for (int v = 0; v < num_vertices; v += 1) {
    boost::add_vertex(graph.local_to_global(v), *subgraphs[labels[v]]);
  }
}

// Returns false iff a feature appears twice in the same image.
bool isConsistent(const Graph& graph) {
  std::set<ImageIndex> visible;

  // Get (begin, end) pair of vertex iterators.
  std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertices;
  vertices = boost::vertices(graph);

  // Iterate through all vertices.
  Graph::vertex_iterator vertex;
  for (vertex = vertices.first; vertex != vertices.second; ++vertex) {
    const FeatureIndex& feature = graph[*vertex];
    ImageIndex frame(feature.view, feature.time);

    // Check if the feature has already been observed in this frame.
    if (visible.find(frame) != visible.end()) {
      // It has! Inconsistent!
      return false;
    }

    // If not then it has now.
    visible.insert(frame);
  }

  return true;
}

void cut(Graph& parent, Graph*& child1, Graph*& child2) {
  int num_vertices = boost::num_vertices(parent);

  child1 = &parent.create_subgraph();
  child2 = &parent.create_subgraph();

  // Spectral clustering.
  std::vector<int> labels;
  bool ok = normalizedCut(parent, labels, FLAGS_max_iter);

  if (ok) {
    for (int j = 0; j < num_vertices; j += 1) {
      if (labels[j] == 0) {
        boost::add_vertex(parent.local_to_global(j), *child1);
      } else {
        boost::add_vertex(parent.local_to_global(j), *child2);
      }
    }
  }
}

void recursiveCut(std::vector<Graph*>& subgraphs) {
  // Copy into stack.
  std::deque<Graph*> deque(subgraphs.begin(), subgraphs.end());
  std::stack<Graph*> pending(deque);
  // Replace with output.
  subgraphs.clear();

  while (!pending.empty()) {
    Graph* subgraph = pending.top();
    pending.pop();

    int num_vertices = boost::num_vertices(*subgraph);

    if (num_vertices > MAX_GRAPH_SIZE) {
      // Too big for the cut algorithm to handle.
      LOG(WARNING) << "Skipping subgraph with too many vertices (" <<
          num_vertices << " > " << MAX_GRAPH_SIZE << ")";
      continue;
    }

    if (num_vertices < MIN_GRAPH_SIZE) {
      // Not enough observations, forget about it.
      DLOG(INFO) << "Skipping subgraph with too few vertices (" <<
          num_vertices << " < " << MIN_GRAPH_SIZE << ")";
      continue;
    }

    if (isConsistent(*subgraph)) {
      // Move to list of valid subgraphs.
      subgraphs.push_back(subgraph);

      DLOG(INFO) << "Found consistent subgraph with " << num_vertices <<
          " vertices";
      continue;
    }

    // Cut the subgraph in two.
    Graph* subgraph1;
    Graph* subgraph2;
    cut(*subgraph, subgraph1, subgraph2);

    int n1 = boost::num_vertices(*subgraph1);
    int n2 = boost::num_vertices(*subgraph2);

    // Add both children.
    if (n1 > 0) {
      // Split into components.
      std::vector<Graph*> children;
      splitIntoComponents(*subgraph1, children);

      std::vector<Graph*>::const_iterator child;
      for (child = children.begin(); child != children.end(); ++child) {
        pending.push(*child);
      }
    }

    if (n2 > 0) {
      // Split into components.
      std::vector<Graph*> children;
      splitIntoComponents(*subgraph2, children);

      std::vector<Graph*>::const_iterator child;
      for (child = children.begin(); child != children.end(); ++child) {
        pending.push(*child);
      }
    }

    DLOG(INFO) << num_vertices << " => {" << n1 << ", " << n2 << "}";
  }
}

void subgraphToTrack(const Graph& subgraph,
                     MultiviewTrack<int>& track,
                     int num_views) {
  // Clear track.
  track = MultiviewTrack<int>(num_views);

  // Iterate over vertices.
  Graph::vertex_iterator begin;
  Graph::vertex_iterator end;
  boost::tie(begin, end) = boost::vertices(subgraph);

  Graph::vertex_iterator vertex;
  for (vertex = begin; vertex != end; ++vertex) {
    // Access feature.
    const FeatureIndex& feature = subgraph[*vertex];

    // Check it doesn't already exist.
    Track<int>& view_track = track.view(feature.view);
    bool exists = (view_track.find(feature.time) != view_track.end());
    CHECK(!exists) << "Multiple observations of one feature in a frame";

    // Set feature index.
    view_track[feature.time] = feature.id;
  }
}

void subgraphsToTracks(const std::vector<Graph*>& subgraphs,
                       MultiviewTrackList<int>& tracks,
                       int num_views) {
  tracks = MultiviewTrackList<int>(num_views);

  std::vector<Graph*>::const_iterator subgraph;
  for (subgraph = subgraphs.begin(); subgraph != subgraphs.end(); ++subgraph) {
    // Create track.
    MultiviewTrack<int> track;
    subgraphToTrack(**subgraph, track, num_views);

    // Swap into list.
    tracks.push_back(MultiviewTrack<int>());
    tracks.back().swap(track);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string matches_format = argv[1];
  std::string view_names_file = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);
  std::string tracks_file = argv[4];

  bool ok;

  std::vector<std::string> views;
  ok = readLines(view_names_file, views);
  int num_views = views.size();
  CHECK(ok) << "Could not load view names";

  // Load matches.
  Graph graph;
  loadAllMatches(matches_format, views, num_frames, graph);
  int num_vertices = boost::num_vertices(graph);
  int num_edges = boost::num_edges(graph);
  LOG(INFO) << "Loaded " << num_edges << " matches between " << num_vertices <<
      " features";

  std::vector<Graph*> subgraphs;
  splitIntoComponents(graph, subgraphs);
  int num_components = subgraphs.size();
  LOG(INFO) << "Found " << num_components << " connected components";

  // Recursively partition.
  recursiveCut(subgraphs);
  LOG(INFO) << "Found " << subgraphs.size() << " valid tracks";

  // Convert each consistent subgraph to a multi-view track of indices.
  MultiviewTrackList<int> tracks;
  subgraphsToTracks(subgraphs, tracks, num_views);

  // Save.
  DefaultWriter<int> writer;
  ok = saveMultiviewTrackList(tracks_file, tracks, writer);
  CHECK(ok) << "Could not save tracks file";
  
  return 0;
}
