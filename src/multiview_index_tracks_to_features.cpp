#include <string>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include "multiview_track_list.hpp"
#include "track_list.hpp"
#include "sift_position.hpp"

#include "read_lines.hpp"
#include "multiview_track_list_reader.hpp"
#include "vector_reader.hpp"
#include "default_reader.hpp"
#include "sift_position_reader.hpp"
#include "track_list_reader.hpp"
#include "multiview_track_list_writer.hpp"
#include "vector_writer.hpp"
#include "sift_position_writer.hpp"

DEFINE_bool(input_multitracks, false,
    "Input tracks or multi-tracks of indices?");
DEFINE_bool(input_keypoints, true, "Build output from keypoints or tracks?");

typedef std::vector<int> IndexSet;
typedef std::vector<SiftPosition> FeatureSet;

std::string makeFrameFilename(const std::string& format,
                              const std::string& view,
                              int time) {
  return boost::str(boost::format(format) % view % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Builds feature tracks from index tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " index-tracks features views num-frames"
      " feature-tracks" << std::endl;
  usage << std::endl;
  usage << "Capable of processing tracks (one feature per frame) or"
      " multi-tracks (set of features per frame). If input_multitracks is false"
      " and input_keypoints is true then the output will be tracks. Otherwise"
      " it will be multi-tracks." << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
void trackToMultitrack(const Track<T>& track,
                       Track<std::vector<T> >& multitrack) {
  typedef std::vector<T> Set;

  // Clear output.
  multitrack.clear();

  typename Track<T>::const_iterator point;

  for (point = track.begin(); point != track.end(); ++point) {
    // Create a size-one set.
    Set set;
    set.push_back(point->second);
    // Add it to the track.
    multitrack[point->first] = set;
  }
}

template<class T>
void multiviewTrackToMultitrack(const MultiviewTrack<T>& track,
                                MultiviewTrack<std::vector<T> >& multitrack) {
  typedef std::vector<T> Set;

  // Clear output structure.
  int num_views = track.numViews();
  multitrack = MultiviewTrack<Set>(num_views);

  // Add point from each view.
  typename MultiviewTrack<T>::const_iterator view_track;
  typename MultiviewTrack<Set>::iterator view_multitrack;

  view_track = track.begin();
  view_multitrack = multitrack.begin();

  for (int i = 0; i < num_views; i += 1) {
    trackToMultitrack(*view_track, *view_multitrack);

    ++view_track;
    ++view_multitrack;
  }
}

template<class T>
void tracksToMultitracks(const MultiviewTrackList<T>& tracks,
                         MultiviewTrackList<std::vector<T> >& multitracks) {
  typedef std::vector<T> Set;

  int num_tracks = tracks.numTracks();
  int num_views = tracks.numViews();

  multitracks = MultiviewTrackList<Set>(num_tracks, num_views);

  for (int i = 0; i < num_tracks; i += 1) {
    // Create multitrack from track.
    MultiviewTrack<Set> multitrack(num_views);
    multiviewTrackToMultitrack(tracks.track(i), multitracks.track(i));
  }
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
void multitrackToTrack(const Track<std::vector<T> >& multitrack,
                       Track<T>& track) {
  typedef std::vector<T> Set;

  track.clear();

  typename Track<Set>::const_iterator it;

  for (it = multitrack.begin(); it != multitrack.end(); ++it) {
    int time = it->first;
    const Set& set = it->second;
    // Assert that the set has exactly one element.
    CHECK(set.size() == 1);
    // Add it to the track.
    track[time] = set.front();
  }
}

template<class T>
void multiviewMultitrackToTrack(
    const MultiviewTrack<std::vector<T> >& multitrack,
    MultiviewTrack<T>& track) {
  typedef std::vector<T> Set;

  int num_views = multitrack.numViews();
  track = MultiviewTrack<T>(num_views);

  // Add point from each view.
  typename MultiviewTrack<Set>::const_iterator view_multitrack;
  typename MultiviewTrack<T>::iterator view_track;

  view_multitrack = multitrack.begin();
  view_track = track.begin();

  for (int i = 0; i < num_views; i += 1) {
    multitrackToTrack(*view_multitrack, *view_track);

    ++view_multitrack;
    ++view_track;
  }
}

template<class T>
void multitracksToTracks(const MultiviewTrackList<std::vector<T> >& multitracks,
                         MultiviewTrackList<T>& tracks) {
  typedef std::vector<T> Set;

  int num_tracks = multitracks.numTracks();
  int num_views = multitracks.numViews();

  tracks = MultiviewTrackList<T>(num_tracks, num_views);

  for (int i = 0; i < num_tracks; i += 1) {
    // Create multitrack from track.
    MultiviewTrack<T> track;
    multiviewMultitrackToTrack(multitracks.track(i), tracks.track(i));
  }
}

////////////////////////////////////////////////////////////////////////////////

bool loadKeypoints(const MultiviewTrackList<IndexSet>& index_tracks,
                   const std::string& features_format,
                   const std::vector<std::string>& views,
                   MultiviewTrackList<FeatureSet>& tracks) {
  // Initialize list of empty tracks.
  int num_features = index_tracks.numTracks();
  int num_views = views.size();
  tracks = MultiviewTrackList<FeatureSet>(num_features, num_views);

  // Iterate through time (to avoid loading all features at once).
  MultiViewTimeIterator<IndexSet> iterator(index_tracks);

  while (!iterator.end()) {
    int time = iterator.time();

    for (int view = 0; view < num_views; view += 1) {
      // Get feature indices matched in this frame.
      std::map<int, IndexSet> subset;
      iterator.getView(view, subset);

      // Only load tracks if there were some features.
      if (!subset.empty()) {
        std::vector<SiftPosition> keypoints;

        // Load features in this frame.
        std::string file;
        file = makeFrameFilename(features_format, views[view], time);
        SiftPositionReader reader;
        bool ok = loadList(file, keypoints, reader);
        if (!ok) {
          return false;
        }
        LOG(INFO) << "Loaded " << keypoints.size() << " features for (" <<
            view << ", " << time << ")";

        // Copy into track.
        std::map<int, IndexSet>::const_iterator it;
        for (it = subset.begin(); it != subset.end(); ++it) {
          int track = it->first;
          const IndexSet& indices = it->second;

          CHECK(track < num_features);

          IndexSet::const_iterator index;
          for (index = indices.begin(); index != indices.end(); ++index) {
            // Instantiates a set if it doesn't already exist.
            FeatureSet& set = tracks.track(track).view(view)[time];

            // Add feature to set.
            CHECK(*index < int(keypoints.size()));
            set.push_back(keypoints[*index]);
          }
        }
      }
    }

    iterator.next();
  }

  return true;
}

bool loadTracks(const MultiviewTrackList<IndexSet>& index_tracks,
                const std::string& features_format,
                const std::vector<std::string>& views,
                MultiviewTrackList<FeatureSet>& tracks) {
  // Initialize list of empty tracks.
  int num_features = index_tracks.numTracks();
  int num_views = views.size();
  tracks = MultiviewTrackList<FeatureSet>(num_views, num_features);

  // Iterate through time (to avoid loading all features at once).
  MultiViewTimeIterator<IndexSet> iterator(index_tracks);

  while (!iterator.end()) {
    int time = iterator.time();

    for (int view = 0; view < num_views; view += 1) {
      // Get feature indices matched in this frame.
      std::map<int, IndexSet> subset;
      iterator.getView(view, subset);

      // Only load tracks if there were some features.
      if (!subset.empty()) {
        TrackList<SiftPosition> features;

        // Load features in this frame.
        std::string file;
        file = makeFrameFilename(features_format, views[view], time);
        SiftPositionReader reader;
        bool ok = loadTrackList(file, features, reader);
        if (!ok) {
          return false;
        }
        LOG(INFO) << "Loaded " << features.size() << " tracks for (" << view <<
            ", " << time << ")";

        // Iterate through features in this frame.
        std::map<int, IndexSet>::const_iterator it;
        for (it = subset.begin(); it != subset.end(); ++it) {
          int id = it->first;
          const IndexSet& indices = it->second;

          IndexSet::const_iterator index;
          for (index = indices.begin(); index != indices.end(); ++index) {
            // Copy every point in the track.
            const Track<SiftPosition>& track = features[*index];

            Track<SiftPosition>::const_iterator point;
            for (point = track.begin(); point != track.end(); ++point) {
              int t = point->first;
              const SiftPosition& x = point->second;

              tracks.track(id).view(view)[t].push_back(x);
            }
          }
        }
      }
    }

    iterator.next();
  }

  return true;
}

int main(int argc, char** argv) {
  init(argc, argv);
  std::string index_tracks_file = argv[1];
  std::string features_format = argv[2];
  std::string views_file = argv[3];
  //int num_frames = boost::lexical_cast<int>(argv[4]);
  std::string feature_tracks_file = argv[5];

  // Load names of views.
  std::vector<std::string> views;
  bool ok = readLines(views_file, views);

  MultiviewTrackList<IndexSet> index_multitracks;

  if (FLAGS_input_multitracks) {
    // Load multi-tracks from file.
    DefaultReader<int> index_reader;
    VectorReader<int> reader(index_reader);
    ok = loadMultiviewTrackList(index_tracks_file, index_multitracks, reader);
    CHECK(ok) << "Could not load multi-tracks";
  } else {
    // Load tracks from file.
    MultiviewTrackList<int> tracks;
    DefaultReader<int> reader;
    ok = loadMultiviewTrackList(index_tracks_file, tracks, reader);
    CHECK(ok) << "Could not load tracks";

    // Convert to multitracks.
    tracksToMultitracks(tracks, index_multitracks);
  }

  MultiviewTrackList<FeatureSet> multitracks;

  // Convert to multitracks of features for reconstructing/visualizing.
  if (FLAGS_input_keypoints) {
    ok = loadKeypoints(index_multitracks, features_format, views, multitracks);
    CHECK(ok) << "Could not load keypoints";
  } else {
    ok = loadTracks(index_multitracks, features_format, views, multitracks);
    CHECK(ok) << "Could not load feature tracks";
  }

  if (!FLAGS_input_multitracks && FLAGS_input_keypoints) {
    // If we input valid tracks and used keypoints not tracks,
    // then we can write out valid tracks.
    MultiviewTrackList<SiftPosition> tracks;
    multitracksToTracks(multitracks, tracks);

    SiftPositionWriter writer;
    saveMultiviewTrackList(feature_tracks_file, tracks, writer);
  } else {
    // Save multitracks.
    SiftPositionWriter feature_writer;
    VectorWriter<SiftPosition> writer(feature_writer);
    saveMultiviewTrackList(feature_tracks_file, multitracks, writer);
  }

  return 0;
}
