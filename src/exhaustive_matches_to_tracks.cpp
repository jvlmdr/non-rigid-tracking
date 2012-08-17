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
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"

#include "match_reader.hpp"
#include "vector_reader.hpp"
#include "read_lines.hpp"
#include "default_writer.hpp"
#include "multiview_track_writer.hpp"
#include "multiview_track_list_writer.hpp"

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

typedef boost::adjacency_list<boost::setS,
                              boost::vecS,
                              boost::undirectedS,
                              FeatureIndex>
        MatchGraph;
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
      Frame frame1(v1, t1);
      Frame frame2(v2, t2);

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
  std::set<Frame> frames;

  for (FeatureList::const_iterator feature = features.begin();
       feature != features.end();
       ++feature) {
    Frame frame(feature->view, feature->time);

    if (frames.find(frame) != frames.end()) {
      // Frame already exists in set.
      return false;
    }

    frames.insert(frame);
  }

  return true;
}

void convertFeaturesToTrack(const FeatureList& features,
                            MultiviewTrack<int>& track,
                            int num_views) {
  // Reset multiview track.
  track.reset(num_views);

  // Assumes features are consistent.
  for (FeatureList::const_iterator feature = features.begin();
       feature != features.end();
       ++feature) {
    track.add(Frame(feature->view, feature->time), feature->id);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string matches_format = argv[1];
  std::string view_names_file = argv[2];
  int num_frames = boost::lexical_cast<int>(argv[3]);
  std::string tracks_file = argv[4];

  bool ok;

  std::vector<std::string> view_names;
  ok = readLines(view_names_file, view_names);
  CHECK(ok) << "Could not load view names";
  int num_views = view_names.size();

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
  std::list<FeatureList> subsets(num_components);

  {
    // Build lookup table.
    std::vector<FeatureList*> subset_map(num_components);

    std::list<FeatureList>::iterator subset = subsets.begin();
    for (int i = 0; i < num_components; i += 1) {
      subset_map[i] = &(*subset);
      ++subset;
    }

    // Put features into their components.
    for (int i = 0; i < num_vertices; i += 1) {
      subset_map[labels[i]]->push_back(graph[i]);
    }
  }

  // Sort through components and ensure that they are consistent.
  {
    std::list<FeatureList>::iterator subset = subsets.begin();
    while (subset != subsets.end()) {
      if (!isConsistent(*subset)) {
        // Inconsistent. Remove it.
        subsets.erase(subset++);
      } else {
        // Consistent. Keep it.
        ++subset;
      }
    }
  }
  LOG(INFO) << "Pruned to " << subsets.size() << " consistent subsets";

  // Convert to tracks!
  MultiviewTrackList<int> tracks(num_views);
  for (std::list<FeatureList>::const_iterator subset = subsets.begin();
       subset != subsets.end();
       ++subset) {
    MultiviewTrack<int> track;
    convertFeaturesToTrack(*subset, track, num_views);

    // Add to list.
    tracks.add(track);
  }

  // Save track list.
  DefaultWriter<int> writer;
  ok = saveMultiviewTrackList(tracks_file, tracks, writer);
  CHECK(ok) << "Could not save tracks";

  return 0;
}
