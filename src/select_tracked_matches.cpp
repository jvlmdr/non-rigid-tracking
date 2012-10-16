#include <vector>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>

#include "match_result.hpp"
#include "sift_position.hpp"
#include "track_list.hpp"

#include "iterator_reader.hpp"
#include "match_result_reader.hpp"
#include "track_list_reader.hpp"
#include "sift_position_reader.hpp"
#include "matrix_reader.hpp"
#include "vector_writer.hpp"
#include "match_result_writer.hpp"

DEFINE_double(max_error, 2., "Maximum deviation from track (pixels)");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Removes matches which are not verified by tracking." << std::endl;
  usage << std::endl;
  usage << argv[0] << " matches tracks1 tracks2 frame-number "
      << "verified-matches" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

bool matchVerified(const Track<SiftPosition>& track1,
                   const Track<SiftPosition>& track2,
                   int t,
                   double max_error) {
  Track<SiftPosition>::const_iterator point1;
  Track<SiftPosition>::const_iterator point2;

  // Find current position in each track.
  point1 = track1.find(t);
  point2 = track2.find(t + 1);
  // These points MUST exist in the track.
  CHECK(point1 != track1.end());
  CHECK(point2 != track2.end());

  cv::Point2d start1(point1->second.x, point1->second.y);
  cv::Point2d start2(point2->second.x, point2->second.y);

  // Find next and previous position, respectively.
  point1 = track1.find(t + 1);
  point2 = track2.find(t);
  // If the track didn't contain this point, then the match is not verified.
  if (point1 == track1.end() || point2 == track2.end()) {
    return false;
  }

  cv::Point2d end1(point1->second.x, point1->second.y);
  cv::Point2d end2(point2->second.x, point2->second.y);

  cv::Point2d d1 = end1 - start2;
  cv::Point2d d2 = end2 - start1;

  // Check that differences were below threshold.
  return (cv::norm(d1) <= max_error && cv::norm(d2) <= max_error);
}

struct MatchNotVerified {
  const TrackList<SiftPosition>* tracks1;
  const TrackList<SiftPosition>* tracks2;
  int t;
  double max_error;

  bool operator()(const MatchResult& match) const {
    return !matchVerified((*tracks1)[match.index1], (*tracks2)[match.index2], t,
        max_error);
  }

  MatchNotVerified(const TrackList<SiftPosition>& tracks1,
                   const TrackList<SiftPosition>& tracks2,
                   int t,
                   double max_error)
      : tracks1(&tracks1), tracks2(&tracks2), t(t), max_error(max_error) {}
};

int main(int argc, char** argv) {
  init(argc, argv);

  std::string matches_file = argv[1];
  std::string tracks_file1 = argv[2];
  std::string tracks_file2 = argv[3];
  int frame_number = boost::lexical_cast<int>(argv[4]);
  std::string verified_file = argv[5];

  bool ok;

  // Load matches.
  std::vector<MatchResult> matches;
  MatchResultReader match_reader;
  ok = loadList(matches_file, matches, match_reader);
  CHECK(ok) << "Could not load matches";

  SiftPositionReader point_reader;

  // Load tracks.
  TrackList<SiftPosition> tracks1;
  TrackList<SiftPosition> tracks2;
  ok = loadTrackList(tracks_file1, tracks1, point_reader);
  CHECK(ok) << "Could not load tracks";
  ok = loadTrackList(tracks_file2, tracks2, point_reader);
  CHECK(ok) << "Could not load tracks";

  std::vector<MatchResult> verified;
  // Construct test.
  MatchNotVerified match_verified(tracks1, tracks2, frame_number,
      FLAGS_max_error);
  // Remove outliers.
  std::remove_copy_if(matches.begin(), matches.end(),
      std::back_inserter(verified), match_verified);

  int num_input = matches.size();
  int num_output = verified.size();
  double fraction = static_cast<double>(num_output) / num_input;
  LOG(INFO) << "Kept " << num_output << " / " << num_input << " matches (" <<
      fraction << ")";

  MatchResultWriter match_writer;
  ok = saveList(verified_file, verified, match_writer);
  CHECK(ok) << "Could not save rigid matches to file";

  return 0;
}
