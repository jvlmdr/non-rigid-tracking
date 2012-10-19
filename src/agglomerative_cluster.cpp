#include <iostream>
#include <string>
#include <sstream>
#include <stack>
#include <map>
#include <set>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/typeof/typeof.hpp>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include "match.hpp"
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"

#include "read_lines.hpp"
#include "multiview_track_list_reader.hpp"
#include "default_reader.hpp"
#include "iterator_reader.hpp"
#include "match_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "vector_writer.hpp"
#include "default_writer.hpp"

// Identifies a feature in a multiview video.
// TODO: Need a better name?
struct FeatureIndex {
  int view;
  int time;
  int id;

  FeatureIndex();
  FeatureIndex(int view, int time, int id);
  FeatureIndex(const Frame& frame, int id);

  // Defines an ordering over feature indices.
  bool operator<(const FeatureIndex& other) const;
};

FeatureIndex::FeatureIndex() : view(-1), time(-1), id(-1) {}

FeatureIndex::FeatureIndex(int view, int time, int id)
    : view(view), time(time), id(id) {}

FeatureIndex::FeatureIndex(const Frame& frame, int id)
    : view(frame.view), time(frame.time), id(id) {}

bool FeatureIndex::operator<(const FeatureIndex& other) const {
  if (view < other.view) {
    return true;
  } else if (other.view < view) {
    return false;
  } else {
    if (time < other.time) {
      return true;
    } else if (other.time < time) {
      return false;
    } else {
      return id < other.id;
    }
  }
}

typedef FeatureIndex VertexProperty;
typedef boost::property<boost::edge_weight_t, int> EdgeProperty;
typedef boost::adjacency_list<boost::vecS,
                              boost::vecS,
                              boost::undirectedS,
                              VertexProperty,
                              EdgeProperty>
        Graph;

typedef Graph::vertex_descriptor Vertex;
typedef std::map<FeatureIndex, Vertex> VertexLookup;
typedef boost::disjoint_sets<int*, Vertex*> DisjointSets;

std::ostream& operator<<(std::ostream& stream, const FeatureIndex& feature) {
  return stream << "(" << feature.view << ", " << feature.time << ", " <<
      feature.id << ")";
}

std::ostream& operator<<(std::ostream& stream, const Frame& frame) {
  return stream << "(" << frame.view << ", " << frame.time << ")";
}

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

Vertex findOrInsert(Graph& graph,
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
                    Graph& graph,
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
      std::vector<Match> match_list;
      MatchReader reader;
      bool ok = loadList(matches_file, match_list, reader);
      CHECK(ok) << "Could not load matches";
      DLOG(INFO) << "Loaded " << match_list.size() << " matches for (" <<
          views[v1] << ", " << t1 << "), (" << views[v2] << ", " << t2 << ")";
      num_matches += match_list.size();

      // Add matches to map.
      Frame frame1(v1, t1);
      Frame frame2(v2, t2);

      for (std::vector<Match>::const_iterator match = match_list.begin();
           match != match_list.end();
           ++match) {
        FeatureIndex feature1(frame1, match->first);
        FeatureIndex feature2(frame2, match->second);

        // Find existing vertex for feature, or insert one.
        Vertex vertex1 = findOrInsert(graph, vertices, feature1);
        Vertex vertex2 = findOrInsert(graph, vertices, feature2);

        // Add vertex to graph. Set edge weight to 1.
        EdgeProperty edge(1);
        boost::add_edge(vertex1, vertex2, edge, graph);
      }
    }
  }
}

void addTracks(const MultiviewTrackList<int>& tracks,
               DisjointSets& sets,
               const VertexLookup& lookup) {
  MultiviewTrackList<int>::const_iterator track;
  for (track = tracks.begin(); track != tracks.end(); ++track) {
    // Merge first vertex with every other.
    bool first = true;
    Vertex first_vertex = 0;

    MultiviewTrack<int>::FeatureIterator iter(*track);
    for (iter.begin(); !iter.end(); iter.next()) {
      Frame frame = iter.get().first;
      int id = *iter.get().second;
      FeatureIndex feature(frame.view, frame.time, id);

      // Find vertex index in reverse lookup.
      VertexLookup::const_iterator entry = lookup.find(feature);
      // Make sure that the entry exists.
      CHECK(entry != lookup.end());
      Vertex vertex = entry->second;

      if (first) {
        first_vertex = vertex;
      } else {
        // Join the two components.
        sets.union_set(first_vertex, vertex);
      }

      first = false;
    }
  }
}

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
  Graph graph;
  VertexLookup lookup;
  loadAllMatches(matches_format, views, num_frames, graph, lookup);
  int num_vertices = boost::num_vertices(graph);
  int num_edges = boost::num_edges(graph);
  LOG(INFO) << "Loaded " << num_edges << " matches between " << num_vertices <<
      " features";

  // Use a disjoint-sets data structure for union-find operations.
  std::vector<int> rank(num_vertices);
  std::vector<Vertex> parent(num_vertices);
  DisjointSets sets(&rank.front(), &parent.front());
  // Assign each vertex to a set.
  boost::initialize_incremental_components(graph, sets);

  // Incorporate initial tracks into components.
  addTracks(initial_tracks, sets, lookup);

  Graph::vertex_iterator begin;
  Graph::vertex_iterator end;
  boost::tie(begin, end) = boost::vertices(graph);
  LOG(INFO) << "Initially " << sets.count_sets(begin, end) << " sets of " <<
      num_vertices << " vertices";

  /*
  // Convert each consistent subgraph to a multi-view track of indices.
  MultiviewTrackList<int> tracks;
  subgraphsToTracks(subgraphs, tracks, num_views);

  // Save.
  DefaultWriter<int> writer;
  ok = saveMultiviewTrackList(tracks_file, tracks, writer);
  CHECK(ok) << "Could not save tracks file";
  */
  
  return 0;
}
