#include <string>
#include <sstream>
#include <stack>
#include <map>
#include <set>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "match.hpp"
#include "track_list.hpp"

#include "match_reader.hpp"
#include "vector_reader.hpp"
#include "read_lines.hpp"

// A video frame is identified by (view, time).
struct FrameIndex {
  int view;
  int time;

  // Defines an ordering over frame indices.
  bool operator<(const FrameIndex& other) const {
    return (view < other.view) || (view == other.view && time < other.time);
  }

  FrameIndex(int view, int time) : view(view), time(time) {}
};

// TODO: A better name?
struct FeatureIndex {
  int view;
  int time;
  int feature;

  bool operator<(const FeatureIndex& other) const {
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
        return feature < other.feature;
      }
    }
  }

  FeatureIndex() : view(-1), time(-1), feature(-1) {}

  FeatureIndex(int view, int time, int feature)
      : view(view), time(time), feature(feature) {}

  FeatureIndex(const FrameIndex& frame, int feature)
      : view(frame.view), time(frame.time), feature(feature) {}
};

typedef boost::adjacency_list<boost::setS,
                              boost::vecS,
                              boost::undirectedS,
                              FeatureIndex>
        MatchGraph;
typedef std::map<FeatureIndex, MatchGraph::vertex_descriptor> VertexLookup;

typedef std::vector<FeatureIndex> FeatureList;

std::ostream& operator<<(std::ostream& stream, const FeatureIndex& index) {
  return stream << "(" << index.view << ", " << index.time << ", " <<
      index.feature << ")";
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Reduces matches to consistent tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " matches-format view-names num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

std::string makeFilename(const std::string& format,
                         const std::string& view1,
                         const std::string& view2,
                         int t1,
                         int t2) {
  return boost::str(
      boost::format(format) % view1 % view2 % (t1 + 1) % (t2 + 1));
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
      std::string matches_file = makeFilename(matches_format, views[v1],
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
      FrameIndex frame1(v1, t1);
      FrameIndex frame2(v2, t2);

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

bool isConsistent(const FeatureList& features) {
  std::set<FrameIndex> frames;

  for (FeatureList::const_iterator feature = features.begin();
       feature != features.end();
       ++feature) {
    FrameIndex frame(feature->view, feature->time);

    if (frames.find(frame) != frames.end()) {
      // Frame already exists in set.
      return false;
    }

    frames.insert(frame);
  }

  return true;
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string matches_format = argv[1];
  std::string view_names_file = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);

  bool ok;

  std::vector<std::string> view_names;
  ok = readLines(view_names_file, view_names);
  CHECK(ok) << "Could not load view names";

  //
  MatchGraph graph;
  loadAllMatches(matches_format, view_names, num_frames, graph);
  int num_vertices = boost::num_vertices(graph);
  int num_edges = boost::num_edges(graph);
  LOG(INFO) << "Loaded " << num_edges << " matches between " << num_vertices <<
      " features";

  // Find connected components of graph.
  std::vector<int> labels(num_vertices);
  int num_components = boost::connected_components(graph, &labels[0]);
  LOG(INFO) << "Found " << num_components << " connected components";

  // Group features into subgraphs.
  std::list<FeatureList> subgraphs(num_components);

  {
    // Build lookup table.
    std::vector<FeatureList*> subgraph_map(num_components);

    std::list<FeatureList>::iterator subgraph = subgraphs.begin();
    for (int i = 0; i < num_components; i += 1) {
      subgraph_map[i] = &(*subgraph);
      ++subgraph;
    }

    // Put features into their components.
    for (int i = 0; i < num_vertices; i += 1) {
      subgraph_map[labels[i]]->push_back(graph[i]);
    }
  }

  // Sort through components and ensure that they are consistent.
  {
    std::list<FeatureList>::iterator subgraph = subgraphs.begin();
    while (subgraph != subgraphs.end()) {
      if (!isConsistent(*subgraph)) {
        // Inconsistent. Remove it.
        subgraphs.erase(subgraph++);
      } else {
        // Consistent. Keep it.
        ++subgraph;
      }
    }
  }
  LOG(INFO) << "Pruned to " << subgraphs.size() << " consistent subgraphs";

  return 0;
}
