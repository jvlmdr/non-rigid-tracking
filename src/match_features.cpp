#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "read_image.hpp"
#include "descriptor.hpp"
#include "keypoint.hpp"
#include "match.hpp"
#include "reader.hpp"
#include "vector_reader.hpp"
#include "writer.hpp"
#include "vector_writer.hpp"

DEFINE_bool(use_flann, true,
    "Use FLANN (fast but approximate) to find nearest neighbours.");
//DEFINE_double(second_best_ratio, 1.0,
//    "Maximum distance relative to second-best match. >= 1 has no effect.");

typedef std::vector<Descriptor> DescriptorList;

// The result of a single match from one image to the other.
typedef cv::DMatch SingleDirectedMatchResult;
// A pair of single matches.
typedef std::vector<SingleDirectedMatchResult> SingleDirectedMatchResultPair;

// The result of a consistent match.
//
// Contains:
// Indices of matching features.
// Distance between matching features.
// Distance to each feature's second-nearest match.
struct MatchResult {
  int index1;
  int index2;
  double dist;
  double second_dist1;
  double second_dist2;
};

class MatchResultWriter : public Writer<MatchResult> {
  public:
    ~MatchResultWriter();
    void write(cv::FileStorage& file, const MatchResult& x);
};

MatchResultWriter::~MatchResultWriter() {}

void MatchResultWriter::write(cv::FileStorage& file,
                              const MatchResult& result) {
  file << "index1" << result.index1;
  file << "index2" << result.index2;
  file << "dist" << result.dist;
  file << "second_dist1" << result.second_dist1;
  file << "second_dist2" << result.second_dist2;
}

typedef std::vector<MatchResult> MatchResultList;

struct DirectedMatchResult {
  int match;
  double dist;
  double second_dist;
};

typedef std::vector<DirectedMatchResult> DirectedMatchResultList;

// Converts a pair of single matches to 
DirectedMatchResult singleDirectedPairToMatchResult(
    const SingleDirectedMatchResultPair& pair) {
  DirectedMatchResult result;

  // Every element in the query set matched to one element in the training set.
  result.match = pair[0].trainIdx;
  result.dist = pair[0].distance;
  result.second_dist = pair[1].distance;

  return result;
}

void listToMatrix(const DescriptorList& list, cv::Mat& matrix) {
  int rows = list.size();
  CHECK(rows != 0);
  int cols = list.front().data.size();
  matrix.create(rows, cols, cv::DataType<float>::type);

  int i = 0;
  DescriptorList::const_iterator descriptor;
  for (descriptor = list.begin(); descriptor != list.end(); ++descriptor) {
    std::copy(descriptor->data.begin(), descriptor->data.end(),
        matrix.row(i).begin<float>());
    i += 1;
  }
}

/*
bool isNotDistinctive(const DirectedMatchResultList& matches, double ratio) {
  DirectedMatchResultList::const_iterator it = matches.begin();
  const cv::DMatch& first = *it;
  ++it;
  const cv::DMatch& second = *it;

  // Note: Carefully crafted to handle case where both distance are zero.
  // (No division. Non-strict comparison.)
  return (first.distance >= ratio * second.distance);
}
*/

void findConsistentMatches(const DirectedMatchResultList& forward_matches,
                           const DirectedMatchResultList& reverse_matches,
                           MatchResultList& matches) {
  matches.clear();

  // Have two lists of pairs (i, j).
  // Only keep pair (i, j) if there exists a pair (j, i) in the other list.
  // The first index i is unique in both lists.

  int num_forward_matches = forward_matches.size();

  for (int i = 0; i < num_forward_matches; i += 1) {
    // Get corresponding reverse match.
    const DirectedMatchResult& forward = forward_matches[i];
    const DirectedMatchResult& reverse = reverse_matches[forward.match];

    if (reverse.match == i) {
      // The reverse match points back. This match was consistent!
      MatchResult match;
      match.index1 = i;
      match.index2 = forward.match;
      match.dist = forward.dist;
      match.second_dist1 = forward.second_dist;
      match.second_dist2 = reverse.second_dist;

      matches.push_back(match);
    }
  }
}

void match(const DescriptorList& descriptors1,
           const DescriptorList& descriptors2,
           MatchResultList& matches,
           bool use_flann) {
  // Load descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(descriptors1, mat1);
  listToMatrix(descriptors2, mat2);

  // Find two nearest neighbours, match in each direction.
  std::vector<SingleDirectedMatchResultPair> forward_match_pairs;
  std::vector<SingleDirectedMatchResultPair> reverse_match_pairs;

  // Construct matcher.
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (use_flann) {
    matcher = cv::DescriptorMatcher::create("FlannBased");
  } else {
    matcher = cv::DescriptorMatcher::create("BruteForce");
  }

  // Match all points in both directions.
  // Get top two matches as this is what we'll be doing in practice.
  matcher->knnMatch(mat1, mat2, forward_match_pairs, 2);
  matcher->knnMatch(mat2, mat1, reverse_match_pairs, 2);

  // Convert pairs to match results.
  DirectedMatchResultList forward_matches;
  DirectedMatchResultList reverse_matches;
  std::transform(forward_match_pairs.begin(), forward_match_pairs.end(),
      std::back_inserter(forward_matches), singleDirectedPairToMatchResult);
  std::transform(reverse_match_pairs.begin(), reverse_match_pairs.end(),
      std::back_inserter(reverse_matches), singleDirectedPairToMatchResult);

  // Reduce to a consistent set.
  findConsistentMatches(forward_matches, reverse_matches, matches);

  LOG(INFO) << matches.size() << " consistent matches";
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Computes matches between sets of descriptors." << std::endl;
  usage << std::endl;
  usage << argv[0] << " descriptors1 descriptors2 matches" << std::endl;
  usage << std::endl;
  usage << "descriptors1, descriptors2 -- Input. Descriptors to match." <<
    std::endl;
  usage << "matches -- Output. Pairwise association of indices." << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string descriptors_file1 = argv[1];
  std::string descriptors_file2 = argv[2];
  std::string matches_file = argv[3];
  //double SECOND_BEST_RATIO = FLAGS_second_best_ratio;

  bool ok;

  // Load descriptors.
  std::vector<Descriptor> descriptors1;
  std::vector<Descriptor> descriptors2;

  DescriptorReader descriptor_reader;
  VectorReader<Descriptor> reader(descriptor_reader);

  LOG(INFO) << "Loading descriptors from `" << descriptors_file1 << "'";
  ok = load(descriptors_file1, descriptors1, reader);
  CHECK(ok) << "Could not load descriptors";
  LOG(INFO) << "Loaded " << descriptors1.size() << " descriptors";

  LOG(INFO) << "Loading descriptors from `" << descriptors_file2 << "'";
  ok = load(descriptors_file2, descriptors2, reader);
  CHECK(ok) << "Could not load descriptors";
  LOG(INFO) << "Loaded " << descriptors2.size() << " descriptors";

  MatchResultList matches;
  match(descriptors1, descriptors2, matches, FLAGS_use_flann);

  MatchResultWriter match_writer;
  VectorWriter<MatchResult> writer(match_writer);
  ok = save(matches_file, matches, writer);
  CHECK(ok) << "Could not save list of matches";

  return 0;
}
