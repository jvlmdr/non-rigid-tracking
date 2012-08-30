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

#include "read_lines.hpp"
#include "vector_reader.hpp"
#include "match_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "vector_writer.hpp"
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

bool multitrackToTrack(const MultiviewTrack<FeatureSet>& multitrack,
                       MultiviewTrack<int>& track) {
  int num_views = multitrack.numViews();
  track.reset(num_views);

  for (int i = 0; i < num_views; i += 1) {
    TrackIterator<FeatureSet> point(multitrack.view(i));

    while (!point.end()) {
      // There should never be an entry with zero features.
      CHECK(point.get().size() > 0);

      if (point.get().size() > 1) {
        // Multiple features in this frame. Inconsistent track!
        return false;
      } else {
        // Add the single element in the vector to the track.
        track.set(Frame(i, point.time()), point.get().front());
      }

      point.next();
    }
  }

  return true;
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
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load matches.
  MatchGraph graph;
  loadAllMatches(matches_format, views, num_frames, graph);
  int num_vertices = boost::num_vertices(graph);
  int num_edges = boost::num_edges(graph);
  LOG(INFO) << "Loaded " << num_edges << " matches between " << num_vertices <<
      " features";

  // Find connected components of graph.
  std::vector<int> labels(num_vertices);
  int num_components = boost::connected_components(graph, &labels.front());
  LOG(INFO) << "Found " << num_components << " connected components";

  // boost::connected_components assigns a label to each node.
  // Now assign features with the same label to one track.

  // "Multitracks" can have more than one feature per frame.
  std::vector<MultiviewTrack<FeatureSet> > multitrack_list(num_components,
      MultiviewTrack<FeatureSet>(num_views));

  for (int i = 0; i < num_vertices; i += 1) {
    const FeatureIndex& vertex = graph[i];
    Frame frame(vertex.view, vertex.time);

    MultiviewTrack<FeatureSet>& multitrack = multitrack_list[labels[i]];

    FeatureSet* set = multitrack.get(frame);

    // If there is no entry for this frame, create a blank one.
    if (set == NULL) {
      multitrack.set(frame, FeatureSet());
      set = multitrack.get(frame);
      CHECK(set != NULL);
    }

    // Add this feature.
    set->push_back(vertex.id);
  }

  MultiviewTrackList<FeatureSet> multitracks(num_views);
  for (int i = 0; i < num_components; i += 1) {
    multitracks.add(multitrack_list[i]);
  }

  if (FLAGS_discard_inconsistent) {
    // Build actual tracks from "multitracks", which have a set per frame.
    MultiviewTrackList<int> tracks(num_views);

    for (int i = 0; i < num_components; i += 1) {
      // Do not keep "multi-tracks" which were observed twice in one frame.
      MultiviewTrack<int> track;
      bool consistent = multitrackToTrack(multitracks.track(i), track);

      if (!consistent) {
        tracks.add(track);
      }
    }

    // Save track list.
    DefaultWriter<int> feature_writer;
    ok = saveMultiviewTrackList(tracks_file, tracks, feature_writer);
    CHECK(ok) << "Could not save tracks";
  } else {
    // Save track list.
    DefaultWriter<int> feature_writer;
    VectorWriter<int> feature_set_writer(feature_writer);
    ok = saveMultiviewTrackList(tracks_file, multitracks, feature_set_writer);
    CHECK(ok) << "Could not save tracks";
  }

  return 0;
}
