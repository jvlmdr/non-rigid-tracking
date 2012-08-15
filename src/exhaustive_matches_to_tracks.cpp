#include <string>
#include <sstream>
#include <stack>
#include <map>
#include <set>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "match.hpp"
#include "track_list.hpp"

#include "match_reader.hpp"
#include "vector_reader.hpp"
#include "read_lines.hpp"

class FeatureIndex;
typedef std::set<FeatureIndex> FeatureSet;
typedef std::map<FeatureIndex, FeatureSet> MatchList;
typedef std::vector<FeatureIndex> FeatureList;

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

  FeatureIndex(int view, int time, int feature)
      : view(view), time(time), feature(feature) {}

  FeatureIndex(const FrameIndex& frame, int feature)
      : view(frame.view), time(frame.time), feature(feature) {}

  // Implicit constructor from mapping of frame to feature.
  //FeatureIndex(const FeaturePerFrame::value_type& mapping)
  //    : view(mapping.first.view),
  //      time(mapping.first.time),
  //      feature(mapping.second) {}
};

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

void addMatch(MatchList& matches,
              const FrameIndex& frame1,
              const FrameIndex& frame2,
              const Match& match) {
  // Add mappings in both directions.
  FeatureIndex feature1(frame1, match.first);
  FeatureIndex feature2(frame2, match.second);

  matches[feature1].insert(feature2);
  matches[feature2].insert(feature1);
}

void loadAllMatches(const std::string& matches_format,
                    const std::vector<std::string>& views,
                    int num_frames,
                    MatchList& matches) {
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
      std::for_each(match_list.begin(), match_list.end(),
          boost::bind(addMatch, boost::ref(matches), frame1, frame2, _1));
    }
  }

  LOG(INFO) << "Loaded " << num_matches << " matches";
}

void findConnectedSubgraphs(MatchList& matches,
                            std::vector<FeatureList>& subgraphs) {
  subgraphs.clear();

  // TODO: There exists some library to do this?

  while (!matches.empty()) {
    // Build list of connected nodes.
    FeatureList connected;

    // Waiting list for nodes to examine.
    std::stack<FeatureIndex> waiting;
    // Add first match to the list of features to look at.
    waiting.push(matches.begin()->first);

    while (!waiting.empty()) {
      // Remove element from waiting list.
      FeatureIndex feature1 = waiting.top();
      waiting.pop();
      // Add to subgraph.
      connected.push_back(feature1);

      // Find matches for this feature.
      MatchList::iterator forward_match = matches.find(feature1);
      // If there are none, then skip.
      if (forward_match == matches.end()) {
        continue;
      }

      // If there are matches, then iterate through them.
      FeatureSet& forward_matches = forward_match->second;

      for (FeatureSet::const_iterator feature2 = forward_matches.begin();
           feature2 != forward_matches.end();
           ++feature2) {
        // Add each to the waiting list.
        waiting.push(*feature2);

        // Find the reverse match (it should be guaranteed to exist).
        MatchList::iterator reverse_match = matches.find(*feature2);
        CHECK(reverse_match != matches.end());
        FeatureSet& reverse_matches = reverse_match->second;
        // Find and erase the match back the other way.
        int num_erased = reverse_matches.erase(feature1);
        CHECK(num_erased == 1);

        // If that was the last match for that feature, remove it entirely.
        // No empty lists allowed.
        if (reverse_matches.empty()) {
          matches.erase(reverse_match);
        }
      }

      matches.erase(forward_match);
    }

    LOG(INFO) << "Found connected set of " << connected.size() << " features";

    subgraphs.push_back(FeatureList());
    subgraphs.back().swap(connected);
  }
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

  int num_views = view_names.size();

  MatchList matches;
  loadAllMatches(matches_format, view_names, num_frames, matches);

  std::vector<FeatureList> subgraphs;
  findConnectedSubgraphs(matches, subgraphs);

  /*
  // Now reduce to consistent matches.
  TrackList<int> tracks;
  findConsistentTracks(matches, tracks);
  */

  return 0;
}
