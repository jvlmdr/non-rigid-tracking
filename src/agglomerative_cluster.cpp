#include <iostream>
#include <string>
#include <sstream>
#include <stack>
#include <map>
#include <set>
#include <algorithm>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <opencv2/core/core.hpp>

#include "match_result.hpp"
#include "multiview_track.hpp"
#include "multiview_track_list.hpp"
#include "feature_index.hpp"
#include "match_graph.hpp"
#include "feature_sets.hpp"
#include "find_smooth_trajectory.hpp"
#include "sift_position.hpp"
#include "camera.hpp"

#include "read_lines.hpp"
#include "multiview_track_list_reader.hpp"
#include "default_reader.hpp"
#include "iterator_reader.hpp"
#include "sift_position_reader.hpp"
#include "match_result_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "camera_reader.hpp"
#include "multiview_track_reader.hpp"

#include "iterator_writer.hpp"
#include "default_writer.hpp"

typedef boost::property_map<MatchGraph, boost::edge_weight_t>::const_type
        EdgeWeightMap;

typedef MatchGraph::vertex_descriptor Vertex;
typedef std::map<FeatureIndex, int> VertexLookup;
typedef std::set<Vertex> VertexSet;

typedef std::vector<SiftPosition> FeatureList;
typedef std::deque<FeatureList> MultiviewFeatureList;
typedef std::deque<MultiviewFeatureList> MultiviewVideoFeatureList;

////////////////////////////////////////////////////////////////////////////////

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Finds tracks by agglomerative clustering." << std::endl;
  usage << std::endl;
  usage << argv[0] << " initial-tracks matches-format keypoints-format "
      "cameras-format view-names num-frames tracks" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 8) {
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

std::string makeImageFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

std::string makeViewFilename(const std::string& format,
                             const std::string& view) {
  return boost::str(boost::format(format) % view);
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
      ImageIndex frame1(v1, t1);
      ImageIndex frame2(v2, t2);

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
                   const std::map<ImageIndex, int>& set,
                   MultiviewTrack<int>& track,
                   int num_views) {
  track = MultiviewTrack<int>(num_views);

  std::map<ImageIndex, int>::const_iterator feature;
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

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

// The properties of joining two sets.
struct TrackProperties {
  double appearance;
  double reprojection_error;
  double non_smoothness;
  double uncertainty;
};

// The properties associated with a set.
struct SetProperties {
  // Each set has an index.
  int index;
  // For each set, store a compatibility with connected sets.
  std::map<int, TrackProperties> merges;
};

TrackProperties computeTrackProperties(
    const MultiviewTrack<PointObservation>& observations) {
  return TrackProperties();
}

TrackProperties computeVertexSetProperties(
    const MultiviewTrack<Camera>& cameras,
    const MultiviewVideoFeatureList& positions,
    const MatchGraph& graph,
    const std::set<int>& vertices) {
  int num_views = cameras.numViews();
  MultiviewTrack<PointObservation> observations(num_views);

  // Assemble cameras and points into "observations" structure.
  std::set<int>::const_iterator vertex;
  for (vertex = vertices.begin(); vertex != vertices.end(); ++vertex) {
    const FeatureIndex& feature = graph[*vertex];

    // Get cameras for this view.
    const Track<Camera>& view_cameras = cameras.view(feature.view);
    // Get camera for this (view, time).
    Track<Camera>::const_iterator camera = view_cameras.find(feature.time);
    // Ensure that the camera is defined.
    if (camera == view_cameras.end()) {
      LOG(WARNING) << "Skipping frame with undefined camera";
    } else {
      cv::Matx34d matrix = camera->second.matrix();

      // Get points for this time.
      CHECK(0 <= feature.time && feature.time < int(positions.size()));
      const MultiviewFeatureList& frame_points = positions[feature.time];
      // Get points for this (view, time).
      CHECK(0 <= feature.view && feature.view < int(frame_points.size()));
      const FeatureList& image_points = frame_points[feature.view];
      // Get point.
      CHECK(0 <= feature.id && feature.id < int(image_points.size()));
      cv::Point2d point = image_points[feature.id].point();

      PointObservation observation;
      observation.P = matrix;
      observation.w = point;

      observations.view(feature.view)[feature.time] = observation;
    }
  }

  return computeTrackProperties(observations);
}

template<class Key, class Value>
void addMapValuesToSet(const std::map<Key, Value>& map, std::set<Value>& set) {
  typename std::map<Key, Value>::const_iterator pair;
  for (pair = map.begin(); pair != map.end(); ++pair) {
    set.insert(pair->second);
  }
}

TrackProperties computeMergeProperties(
    const MultiviewTrack<Camera>& cameras,
    const MultiviewVideoFeatureList& positions,
    const MatchGraph& graph,
    const std::map<ImageIndex, int>& vertices1,
    const std::map<ImageIndex, int>& vertices2) {
  std::set<int> vertices;

  addMapValuesToSet(vertices1, vertices);
  addMapValuesToSet(vertices1, vertices);

  return computeVertexSetProperties(cameras, positions, graph, vertices);
}

bool loadFeatures(const std::string& view,
                  int time,
                  FeatureList& features,
                  const std::string& format) {
  features.clear();
  std::string file = makeImageFilename(format, view, time);
  SiftPositionReader reader;
  return loadList(file, features, reader);
}

bool loadMultiviewFeatures(const std::vector<std::string>& views,
                           int time,
                           MultiviewFeatureList& multiview_features,
                           const std::string& format) {
  multiview_features.clear();

  std::vector<std::string>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view) {
    // Load features for this (view, time).
    FeatureList features;
    bool ok = loadFeatures(*view, time, features, format);
    if (!ok) {
      return false;
    }

    // Swap into list.
    multiview_features.push_back(FeatureList());
    multiview_features.back().swap(features);
  }

  return true;
}

bool loadMultiviewVideoFeatures(const std::vector<std::string>& views,
                                int num_frames,
                                MultiviewVideoFeatureList& video_features,
                                const std::string& format) {
  video_features.clear();

  for (int t = 0; t < num_frames; t += 1) {
    // Load features for this time.
    MultiviewFeatureList features;
    bool ok = loadMultiviewFeatures(views, t, features, format);
    if (!ok) {
      return false;
    }

    // Swap into list.
    video_features.push_back(MultiviewFeatureList());
    video_features.back().swap(features);
  }

  return true;
}

bool loadCameras(const std::vector<std::string>& views,
                 const std::string& format,
                 MultiviewTrack<Camera>& multiview_cameras) {
  int num_views = views.size();
  multiview_cameras = MultiviewTrack<Camera>(num_views);

  std::vector<std::string>::const_iterator view;
  int index = 0;

  for (view = views.begin(); view != views.end(); ++view) {
    // Construct filename.
    std::string file = makeViewFilename(format, *view);

    // Load cameras for this view.
    Track<Camera> cameras;
    CameraReader camera_reader;
    TrackReader<Camera> track_reader(camera_reader);
    bool ok = load(file, cameras, track_reader);
    if (!ok) {
      return false;
    }

    // Swap into multiview structure.
    multiview_cameras.view(index).swap(cameras);

    index += 1;
  }

  return true;
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string initial_tracks_file = argv[1];
  std::string matches_format = argv[2];
  std::string keypoints_format = argv[3];
  std::string cameras_format = argv[4];
  std::string view_names_file = argv[5];
  int num_frames = boost::lexical_cast<int>(argv[6]);
  std::string tracks_file = argv[7];

  bool ok;

  // Load view names.
  std::vector<std::string> views;
  ok = readLines(view_names_file, views);
  CHECK(ok) << "Could not load view names";
  int num_views = views.size();

  // Load position of every feature in every frame.
  LOG(INFO) << "Loading keypoints for all frames";
  MultiviewVideoFeatureList positions;
  ok = loadMultiviewVideoFeatures(views, num_frames, positions,
      keypoints_format);
  CHECK(ok) << "Could not load keypoints";

  // Load cameras.
  MultiviewTrack<Camera> cameras;
  LOG(INFO) << "Loading cameras for all frames";
  ok = loadCameras(views, cameras_format, cameras);
  CHECK(ok) << "Could not load cameras";

  // Load initial tracks.
  MultiviewTrackList<int> initial_tracks;
  DefaultReader<int> int_reader;
  ok = loadMultiviewTrackList(initial_tracks_file, initial_tracks, int_reader);
  CHECK(ok) << "Could not load initial tracks";

  // Load all matches into graph.
  MatchGraph graph;
  VertexLookup lookup;
  LOG(INFO) << "Loading matches between image";
  loadAllMatches(matches_format, views, num_frames, graph, lookup);
  int num_vertices = boost::num_vertices(graph);
  int num_edges = boost::num_edges(graph);
  LOG(INFO) << "Loaded " << num_edges << " matches between " << num_vertices <<
      " features";

  FeatureSets<SetProperties> sets;

  // Build a list containing the frame of every vertex.
  std::vector<ImageIndex> frames;
  {
    MatchGraph::vertex_iterator vertex;
    MatchGraph::vertex_iterator end;
    boost::tie(vertex, end) = boost::vertices(graph);

    for(; vertex != end; ++vertex) {
      const FeatureIndex& feature = graph[*vertex];
      frames.push_back(ImageIndex(feature.view, feature.time));
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

  // Initialize index of every set.
  LOG(INFO) << "Initializing set indices";
  {
    FeatureSets<SetProperties>::const_iterator set;
    int index = 0;

    for (set = sets.begin(); set != sets.end(); ++set) {
      // Get one vertex in the set.
      int vertex = set->second.elements.begin()->second;
      // Obtain write-access to the set properties using the vertex.
      SetProperties& properties = sets.property(vertex);

      properties.index = index;

      index += 1;
    }
  }

  // Set the appearance of each pair of sets to the minimum edge distance.
  LOG(INFO) << "Initializing connectivity of sets";
  {
    std::vector<Edge>::const_iterator edge;
    for (edge = edges.begin(); edge != edges.end(); ++edge) {
      // Only proceed if the edge joins two sets.
      if (!sets.together(edge->source, edge->target)) {
        // Get the two sets which the edge joins.
        SetProperties& properties1 = sets.property(edge->source);
        SetProperties& properties2 = sets.property(edge->target);

        std::map<int, TrackProperties>& merges1 = properties1.merges;
        std::map<int, TrackProperties>& merges2 = properties2.merges;

        // Check if the two sets have already been connected by an edge.
        if (merges1.find(properties2.index) == merges1.end()) {
          // Not connected yet.
          CHECK(merges2.find(properties1.index) == merges2.end());

          // Compute merge properties.
          TrackProperties properties = computeMergeProperties(cameras,
              positions, graph, sets.find(edge->source).elements,
              sets.find(edge->target).elements);

          // Create connection.
          TrackProperties& merge1 = merges1[properties2.index];
          TrackProperties& merge2 = merges2[properties1.index];

          merge1.appearance = edge->weight;
          merge2.appearance = edge->weight;
        } else {
          // Already connected.
          CHECK(merges2.find(properties1.index) != merges2.end());
          TrackProperties& merge1 = merges1[properties2.index];
          TrackProperties& merge2 = merges2[properties1.index];

          // Check if this appearance is better.
          CHECK(merge1.appearance == merge2.appearance);
          if (edge->weight < merge1.appearance) {
            merge1.appearance = edge->weight;
            merge2.appearance = edge->weight;
          }
        }
      }
    }
  }

  LOG(INFO) << "Building heap";
  std::make_heap(edges.begin(), edges.end());

  LOG(INFO) << "Begin clustering";
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
