#include <iostream>
#include <fstream>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "tools/using.hpp"
#include "videoseg/segmentation.hpp"
#include "videoseg/region.hpp"
#include "util/random-color.hpp"
#include "tracking/track-list.hpp"
#include <google/protobuf/repeated_field.h>

using google::protobuf::RepeatedPtrField;
using tracking::TrackList;
using videoseg::VideoSegmentation;
using videoseg::Rasterization;

DEFINE_double(min_fraction, 1.,
    "The fraction of frames that need to be inside the foreground region");

const int FOREGROUND_LABEL = 0;
const int BACKGROUND_LABEL = 1;

typedef RepeatedPtrField<TrackList::Frame> TrackFrameList;
typedef RepeatedPtrField<TrackList::Point> PointList;
typedef RepeatedPtrField<VideoSegmentation::Frame> SegmentationFrameList;
typedef RepeatedPtrField<VideoSegmentation::Frame::Region> RegionList;

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Applies a foreground mask to a list of tracks." << std::endl;
  usage << std::endl;
  usage << argv[0] << " tracks foreground filtered-tracks" << std::endl;
  usage << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

struct Tally {
  int num_inside;
  int total;

  Tally() : num_inside(0), total(0) {}
};

void updateStatistics(const RepeatedPtrField<TrackList::Point>& points,
                      const Rasterization& foreground,
                      map<int, Tally>& stats) {

  PointList::const_iterator point;
  for (point = points.begin(); point != points.end(); ++point) {
    // Get existing element, or insert one.
    Tally& tally = stats[point->id()];
    tally.total += 1;

    // Round to nearest integer coordinates.
    cv::Point pos(std::floor(point->x() + 0.5), std::floor(point->y() + 0.5));

    if (regionContains(foreground, pos)) {
      tally.num_inside += 1;
    }
  }
}

void computeStatistics(const TrackList& tracks,
                       const VideoSegmentation& segmentation,
                       map<int, Tally>& stats) {
  const TrackFrameList& track_frames = tracks.frames();
  const SegmentationFrameList& seg_frames = segmentation.frames();

  // Segmentation does not include first or last frame.
  CHECK_EQ(track_frames.size(), seg_frames.size() + 2);

  TrackFrameList::const_iterator track_frame = track_frames.begin();
  SegmentationFrameList::const_iterator seg_frame = seg_frames.begin();
  ++track_frame;

  // Iterate through frames.
  while (seg_frame != seg_frames.end()) {
    // Find foreground region in list ordered by ID.
    const RegionList& regions = seg_frame->regions();
    RegionList::const_iterator region = std::lower_bound(regions.begin(),
        regions.end(), FOREGROUND_LABEL, videoseg::RegionBefore());

    if (region != regions.end() && region->id() == FOREGROUND_LABEL) {
      // Found a foreground region in this frame. Update statistics.
      updateStatistics(track_frame->points(), region->raster(), stats);
    }

    ++track_frame;
    ++seg_frame;
  }
}

void findSubset(const map<int, Tally>& stats,
                set<int>& ids,
                double min_fraction) {
  map<int, Tally>::const_iterator track;

  for (track = stats.begin(); track != stats.end(); ++track) {
    int id = track->first;
    const Tally& tally = track->second;

    double fraction = double(tally.num_inside) / tally.total;

    if (fraction >= min_fraction) {
      // Keep track.
      ids.insert(id);
    }
  }
}

void selectSubset(const TrackList& tracks,
                  const set<int>& ids,
                  TrackList& subset) {
  const TrackFrameList& frames = tracks.frames();

  TrackFrameList::const_iterator frame;
  for (frame = frames.begin(); frame != frames.end(); ++frame) {
    const PointList& points = frame->points();
    TrackList::Frame new_frame;

    // Points in the frame and indices in the set are ordered.
    set<int>::const_iterator id = ids.begin();
    int new_id = 0;
    PointList::const_iterator point = points.begin();

    while (id != ids.end() && point != points.end()) {
      if (*id < point->id()) {
        ++id;
        new_id += 1;
      } else if (point->id() < *id) {
        ++point;
      } else {
        // Have a match. Adjust the ID and copy the point.
        TrackList::Point new_point(*point);
        new_point.set_id(new_id);
        new_frame.mutable_points()->Add()->Swap(&new_point);

        // Next point in the set.
        ++id;
        new_id += 1;
        // Next point in the frame.
        ++point;
      }
    }

    // Add frame to the track list.
    subset.mutable_frames()->Add()->Swap(&new_frame);
  }
}

void selectForegroundTracks(const TrackList& tracks,
                            const VideoSegmentation& segmentation,
                            TrackList& foreground_tracks,
                            double min_fraction) {
  map<int, Tally> stats;
  computeStatistics(tracks, segmentation, stats);

  set<int> foreground_ids;
  findSubset(stats, foreground_ids, min_fraction);

  // Print number of tracks.
  int kept = std::floor(100. * foreground_ids.size() / stats.size() + 0.5);
  std::cout << "Keeping " << foreground_ids.size() << " / " << stats.size() <<
      " tracks (" << kept << "%)" << std::endl;

  selectSubset(tracks, foreground_ids, foreground_tracks);
}

int main(int argc, char** argv) {
  init(argc, argv);

  string input_tracks_file = argv[1];
  string foreground_file = argv[2];
  string out_tracks_file = argv[3];

  bool ok;

  // Load tracks from file.
  TrackList input_tracks;
  {
    std::ifstream ifs(input_tracks_file.c_str(), std::ios::binary);
    if (!ifs) {
      LOG(FATAL) << "Could not open tracks file";
    }
    ok = input_tracks.ParseFromIstream(&ifs);
    if (!ok) {
      LOG(FATAL) << "Could not parse tracks from file";
    }
  }

  // Load segmentation from file.
  VideoSegmentation segmentation;
  {
    std::ifstream ifs(foreground_file.c_str(), std::ios::binary);
    if (!ifs) {
      LOG(FATAL) << "Could not open segmentation file";
    }
    ok = segmentation.ParseFromIstream(&ifs);
    if (!ok) {
      LOG(FATAL) << "Could not parse segmentation from file";
    }
  }

  TrackList output_tracks;
  selectForegroundTracks(input_tracks, segmentation, output_tracks,
      FLAGS_min_fraction);

  std::ofstream ofs(out_tracks_file.c_str(),
      std::ios::trunc | std::ios::binary);
  ok = output_tracks.SerializeToOstream(&ofs);
  if (!ok) {
    LOG(FATAL) << "Could not save output";
  }

  return 0;
}
