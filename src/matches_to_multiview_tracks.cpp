#include <string>
#include <sstream>
#include <stack>
#include <map>
#include <set>
#include <utility>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "match.hpp"
#include "feature_index.hpp"
#include "match_graph.hpp"
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

DEFINE_bool(exhaustive, false, "Match all pairs of images.");
DEFINE_bool(simultaneous, false,
    "Only match between images taken at the same time.");
DEFINE_bool(adjacent, false,
    "Match image to all views at the same time and its own view in adjacent times.");
DEFINE_bool(one_to_all, false, "Match one image to all others.");

DEFINE_int32(one_to_all_view, -1, "View of image.");
DEFINE_int32(one_to_all_time, -1, "Time of image.");

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

// For when more than one feature may be observed in each frame.
typedef std::vector<int> FeatureSet;

typedef std::map<FeatureIndex, MatchGraph::vertex_descriptor> VertexLookup;
typedef std::vector<FeatureIndex> FeatureList;

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

typedef std::pair<ImageIndex, ImageIndex> ImagePair;

void appendSimultaneousImagePairs(int num_views,
                                  int num_frames,
                                  std::set<ImagePair>& pairs) {
  // For every frame...
  for (int t = 0; t < num_frames; t += 1) {
    // ...take all unordered pairs of views.
    for (int p = 0; p < num_views; p += 1) {
      for (int q = p + 1; q < num_views; q += 1) {
        pairs.insert(ImagePair(ImageIndex(p, t), ImageIndex(q, t)));
      }
    }
  }
}

void appendAdjacentImagePairs(int num_views,
                              int num_frames,
                              std::set<ImagePair>& pairs) {
  // For all views...
  for (int v = 0; v < num_views; v += 1) {
    // ...match between adjacent frames.
    for (int t = 0; t < num_frames - 1; t += 1) {
      pairs.insert(ImagePair(ImageIndex(v, t), ImageIndex(v, t + 1)));
    }
  }
}

void appendAllImagePairs(int num_views,
                         int num_frames,
                         std::set<ImagePair>& pairs) {
  for (int t = 0; t < num_frames; t += 1) {
    for (int u = t + 1; u < num_frames; u += 1) {
      for (int v = 0; v < num_views; v += 1) {
        for (int w = v + 1; w < num_views; w += 1) {
          pairs.insert(ImagePair(ImageIndex(v, t), ImageIndex(w, u)));
        }
      }
    }
  }
}

void appendImagePairsContaining(const ImageIndex& image,
                                int num_views,
                                int num_frames,
                                std::set<ImagePair>& pairs) {
  for (int t = 0; t < num_frames; t += 1) {
    for (int v = 0; v < num_views; v += 1) {
      ImageIndex other(v, t);

      // Make sure the least goes first.
      if (other < image) {
        pairs.insert(ImagePair(other, image));
      } else if (image < other) {
        pairs.insert(ImagePair(image, other));
      }
    }
  }
}

void loadMatches(const std::string& format,
                 const std::vector<std::string>& views,
                 const ImageIndex& image1,
                 const ImageIndex& image2,
                 MatchGraph& graph,
                 VertexLookup& vertices) {
  // Construct filename.
  const std::string& view1 = views[image1.view];
  const std::string& view2 = views[image2.view];
  int time1 = image1.time;
  int time2 = image2.time;
  std::string file = makeMatchFilename(format, view1, view2, time1, time2);

  // Load matches from file.
  std::vector<Match> matches;
  MatchReader reader;
  bool ok = loadList(file, matches, reader);
  CHECK(ok) << "Could not load matches";
  DLOG(INFO) << "Loaded " << matches.size() << " matches for (" << image1 <<
      ", " << image2 << ")";

  std::vector<Match>::const_iterator match;
  for (match = matches.begin(); match != matches.end(); ++match) {
    FeatureIndex feature1(image1, match->first);
    FeatureIndex feature2(image2, match->second);

    // Find existing vertex for feature, or insert one.
    MatchGraph::vertex_descriptor vertex1;
    MatchGraph::vertex_descriptor vertex2;
    vertex1 = findOrInsert(graph, vertices, feature1);
    vertex2 = findOrInsert(graph, vertices, feature2);

    // Add vertex to graph.
    boost::add_edge(vertex1, vertex2, graph);
  }
}

void loadAllMatches(const std::string& format,
                    const std::vector<std::string>& views,
                    const std::set<ImagePair>& image_pairs,
                    MatchGraph& graph,
                    VertexLookup& vertices) {
  vertices.clear();

  std::set<ImagePair>::const_iterator pair;
  for (pair = image_pairs.begin(); pair != image_pairs.end(); ++pair) {
    loadMatches(format, views, pair->first, pair->second, graph, vertices);
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

  // Which image pairs to use?
  std::set<ImagePair> pairs;
  if (FLAGS_exhaustive) {
    // Append all pairs!
    appendAllImagePairs(num_views, num_frames, pairs);
  } else if (FLAGS_simultaneous || FLAGS_adjacent) {
    // These flags can be combined.
    if (FLAGS_simultaneous) {
      appendSimultaneousImagePairs(num_views, num_frames, pairs);
    }
    if (FLAGS_adjacent) {
      appendAllImagePairs(num_views, num_frames, pairs);
    }
  } else if (FLAGS_one_to_all) {
    // Check that view and time parameters were supplied.
    int view = FLAGS_one_to_all_view;
    int time = FLAGS_one_to_all_time;
    CHECK(0 <= view && view < num_views);
    CHECK(0 <= time && time < num_frames);
    // Append pairs containing this image.
    ImageIndex image(view, time);
    appendImagePairsContaining(image, num_views, num_frames, pairs);
  } else {
    LOG(WARNING) << "No image pairs specified";
  }

  // Load matches.
  MatchGraph graph;
  VertexLookup vertices;
  loadAllMatches(matches_format, views, pairs, graph, vertices);
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
    ImageIndex frame(vertex.view, vertex.time);

    MultiviewTrack<FeatureSet>& multitrack = multitrack_list[labels[i]];

    FeatureSet* set = multitrack.point(frame);

    // If there is no entry for this frame, create a blank one.
    if (set == NULL) {
      multitrack.view(vertex.view)[vertex.time] = FeatureSet();
      set = multitrack.point(frame);
      CHECK(set != NULL);
    }

    // Add this feature.
    set->push_back(vertex.id);
  }

  MultiviewTrackList<FeatureSet> multitracks(num_views);
  for (int i = 0; i < num_components; i += 1) {
    multitracks.push_back(MultiviewTrack<FeatureSet>());
    multitracks.back().swap(multitrack_list[i]);
  }

  if (FLAGS_discard_inconsistent) {
    // Build actual tracks from "multitracks", which have a set per frame.
    MultiviewTrackList<int> tracks(num_views);

    for (int i = 0; i < num_components; i += 1) {
      // Do not keep "multi-tracks" which were observed twice in one frame.
      MultiviewTrack<int> track;
      bool consistent = multitrackToTrack(multitracks.track(i), track);

      if (consistent) {
        // Add to the list.
        tracks.push_back(MultiviewTrack<int>());
        tracks.back().swap(track);
      }
    }

    int num_tracks_discarded = multitracks.numTracks() - tracks.numTracks();
    LOG(INFO) << "Discarded " << num_tracks_discarded <<
        " inconsistent tracks (" << tracks.numTracks() << "/" <<
        multitracks.numTracks() << " remain)";

    // More interested in the number of features we threw out!
    int num_features_discarded = multitracks.numImageFeatures() -
      tracks.numImageFeatures();
    LOG(INFO) << "Discarded " << num_features_discarded <<
        " features (" << tracks.numImageFeatures() << "/" <<
        multitracks.numImageFeatures() << " remain)";

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
