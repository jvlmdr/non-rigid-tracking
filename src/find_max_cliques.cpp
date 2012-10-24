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
#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "match.hpp"
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"

#include "read_lines.hpp"
#include "iterator_reader.hpp"
#include "match_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "iterator_writer.hpp"
#include "default_writer.hpp"

DEFINE_bool(discard_inconsistent, false,
    "Discard any feature which appears twice in one frame.");

// More than one feature may be observed in each frame.
typedef std::vector<int> FeatureSet;

// Identifies a feature in a multiview video.
// TODO: Need a better name?
struct FeatureIndex {
  int view;
  int time;
  int id;

  FeatureIndex();
  FeatureIndex(int view, int time, int id);
  FeatureIndex(const ImageIndex& frame, int id);

  // Defines an ordering over feature indices.
  bool operator<(const FeatureIndex& other) const;
};

FeatureIndex::FeatureIndex() : view(-1), time(-1), id(-1) {}

FeatureIndex::FeatureIndex(int view, int time, int id)
    : view(view), time(time), id(id) {}

FeatureIndex::FeatureIndex(const ImageIndex& frame, int id)
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

typedef boost::undirected_graph<FeatureIndex> MatchGraph;
typedef std::map<FeatureIndex, MatchGraph::vertex_descriptor> VertexLookup;
typedef std::vector<FeatureIndex> FeatureList;

std::ostream& operator<<(std::ostream& stream, const FeatureIndex& feature) {
  return stream << "(" << feature.view << ", " << feature.time << ", " <<
      feature.id << ")";
}

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

MatchGraph::vertex_descriptor findOrInsert(MatchGraph& graph,
                                           VertexLookup& vertices,
                                           const FeatureIndex& feature) {
  MatchGraph::vertex_descriptor vertex;

  // Try to find feature in lookup map.
  VertexLookup::const_iterator mapping = vertices.find(feature);

  if (mapping != vertices.end()) {
    // Found the vertex, it already exists.
    vertex = mapping->second;
  } else {
    // Did not find it, create an entry.
    vertex = boost::add_vertex(feature, graph);
    vertices[feature] = vertex;
  }

  return vertex;
}

void loadAllMatches(const std::string& matches_format,
                    const std::vector<std::string>& views,
                    int num_frames,
                    MatchGraph& graph) {
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
        MatchGraph::vertex_descriptor vertex1;
        MatchGraph::vertex_descriptor vertex2;
        vertex1 = findOrInsert(graph, vertices, feature1);
        vertex2 = findOrInsert(graph, vertices, feature2);

        // Add vertex to graph.
        boost::add_edge(vertex1, vertex2, graph);
      }
    }
  }
}

bool multitrackToTrack(const MultiviewTrack<FeatureSet>& multiview_multitrack,
                       MultiviewTrack<int>& multiview_track) {
  int num_views = multiview_multitrack.numViews();
  multiview_track = MultiviewTrack<int>(num_views);

  MultiviewTrack<FeatureSet>::const_iterator multitrack;
  MultiviewTrack<int>::iterator track;

  multitrack = multiview_multitrack.begin();
  track = multiview_track.begin();

  while (multitrack != multiview_multitrack.end()) {
    TrackIterator<FeatureSet> point(*multitrack);

    while (!point.end()) {
      // There should never be an entry with zero features.
      CHECK(point.get().size() > 0);

      if (point.get().size() > 1) {
        // Multiple features in this frame. Inconsistent track!
        return false;
      } else {
        // Add the single element in the vector to the track.
        (*track)[point.time()] = point.get().front();
      }

      point.next();
    }

    ++multitrack;
    ++track;
  }

  return true;
}

// The clique_printer is a visitor that will print the vertices that comprise
// a clique. Note that the vertices are not given in any specific order.
template <typename OutputStream>
struct clique_printer
{
    clique_printer(OutputStream& stream)
        : os(stream)
    { }

    template <typename Clique, typename Graph>
    void clique(const Clique& c, const Graph& g)
    {
        // Iterate over the clique and print each vertex within it.
        typename Clique::const_iterator i, end = c.end();
        for(i = c.begin(); i != end; ++i) {
            os << g[*i] << " ";
        }
        os << std::endl;
    }
    OutputStream& os;
};

int main(int argc, char** argv) {
  init(argc, argv);

  std::string matches_format = argv[1];
  std::string view_names_file = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);
  std::string tracks_file = argv[4];

  bool ok;

  std::vector<std::string> views;
  ok = readLines(view_names_file, views);
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load matches.
  MatchGraph graph;
  loadAllMatches(matches_format, views, num_frames, graph);
  int num_vertices = boost::num_vertices(graph);
  int num_edges = boost::num_edges(graph);
  LOG(INFO) << "Loaded " << num_edges << " matches between " << num_vertices <<
      " features";

  // Instantiate the visitor for printing cliques
  clique_printer<std::ostream> vis(std::cout);

  // Use the Bron-Kerbosch algorithm to find all cliques, printing them
  // as they are found.
  boost::bron_kerbosch_all_cliques(graph, vis, 8);

  return 0;
}
