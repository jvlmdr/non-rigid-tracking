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

// List of descriptors.
typedef std::vector<Descriptor> DescriptorList;

// List of matches with detailed results.
typedef std::vector<cv::DMatch> MatchResultList;

void listToMatrix(const DescriptorList& list, cv::Mat& matrix) {
  int rows = list.size();
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

bool isNotDistinctive(const MatchResultList& matches, double ratio) {
  MatchResultList::const_iterator it = matches.begin();
  const cv::DMatch& first = *it;
  ++it;
  const cv::DMatch& second = *it;

  // Note: Carefully crafted to handle case where both distance are zero.
  // (No division. Non-strict comparison.)
  return (first.distance >= ratio * second.distance);
}

Match extractIndices(const cv::DMatch& match) {
  return Match(match.queryIdx, match.trainIdx);
}

Match extractFirstMatch(const MatchResultList& matches) {
  return extractIndices(matches.front());
}

struct IsInconsistent {
  typedef std::map<int, int> Lookup;
  Lookup lookup;

  bool operator()(const Match& forward) {
    // Look up (second -> first) in reverse map.
    Lookup::const_iterator reverse = lookup.find(forward.second);

    if (reverse == lookup.end()) {
      // No entry. Inconsistent.
      return true;
    } else if (reverse->second != forward.first) {
      // Different index. Inconsistent.
      return true;
    } else {
      // Matching index. Consistent.
      return false;
    }
  }

  IsInconsistent(const MatchList& matches) {
    // Construct map for indexed lookup.
    std::copy(matches.begin(), matches.end(),
        std::inserter(lookup, lookup.begin()));
  }
};

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Evaluates effect of SIFT threshold on matches and speed." <<
    std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " descriptors1 descriptors2" << std::endl;
  usage << std::endl;
  usage << "Parameters:" << std::endl;
  usage << "descriptors1, descriptors2 -- Input. Descriptors to match." <<
    std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string descriptors_file1 = argv[1];
  std::string descriptors_file2 = argv[2];

  bool ok;

  // Load descriptors.
  DescriptorList descriptors1;
  DescriptorList descriptors2;

  ok = loadDescriptors(descriptors_file1, descriptors1);
  CHECK(ok) << "Could not load descriptors!";

  ok = loadDescriptors(descriptors_file2, descriptors2);
  CHECK(ok) << "Could not load descriptors!";

  // Load descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(descriptors1, mat1);
  listToMatrix(descriptors2, mat2);

  // Create matcher.
  cv::Ptr<cv::DescriptorMatcher> matcher;
  matcher = cv::DescriptorMatcher::create("FlannBased");

  // Find two nearest neighbours, match in each direction.
  std::vector<MatchResultList> forward_match_lists;
  std::vector<MatchResultList> reverse_match_lists;

  std::cerr << "Forward matching..." << std::endl;
  matcher->knnMatch(mat1, mat2, forward_match_lists, 2);
  std::cerr << forward_match_lists.size() << " matches found" << std::endl;

  std::cerr << "Reverse matching..." << std::endl;
  matcher->knnMatch(mat2, mat1, reverse_match_lists, 2);
  std::cerr << reverse_match_lists.size() << " matches found" << std::endl;

  /*
  // Prune under a certain threshold.
  std::cerr << "Enforcing distinctive forward matches..." << std::endl;
  {
    std::vector<MatchResultList> distinctive;
    std::remove_copy_if(forward_match_lists.begin(), forward_match_lists.end(),
        std::back_inserter(distinctive),
        boost::bind(isNotDistinctive, _1, min_relative_distance));
    forward_match_lists.swap(distinctive);
  }
  std::cerr << forward_match_lists.size() << " forward matches remain" <<
    std::endl;

  std::cerr << "Enforcing distinctive reverse matches..." << std::endl;
  {
    std::vector<MatchResultList> distinctive;
    std::remove_copy_if(reverse_match_lists.begin(), reverse_match_lists.end(),
        std::back_inserter(distinctive),
        boost::bind(isNotDistinctive, _1, min_relative_distance));
    reverse_match_lists.swap(distinctive);
  }
  std::cerr << reverse_match_lists.size() << " reverse matches remain" <<
    std::endl;
  */

  // Keep only first match.
  MatchList forward_matches;
  MatchList reverse_matches;
  std::transform(forward_match_lists.begin(), forward_match_lists.end(),
      std::back_inserter(forward_matches), extractFirstMatch);
  std::transform(reverse_match_lists.begin(), reverse_match_lists.end(),
      std::back_inserter(reverse_matches), extractFirstMatch);

  // Now check consistency in both directions.
  std::cerr << "Enforcing consistency..." << std::endl;
  MatchList matches;
  std::remove_copy_if(forward_matches.begin(), forward_matches.end(),
      std::back_inserter(matches), IsInconsistent(reverse_matches));
  std::cerr << matches.size() << " matches remain" << std::endl;

  return 0;
}
