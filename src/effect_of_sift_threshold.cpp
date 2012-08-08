#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "read_image.hpp"
#include "descriptor.hpp"
#include "keypoint.hpp"
#include "match.hpp"
#include "vector_reader.hpp"

typedef std::vector<Descriptor> DescriptorList;
typedef std::vector<cv::DMatch> MatchResultList;
typedef std::vector<Match> MatchList;

std::string makeFilename(const std::string& format, double threshold, int n) {
  return boost::str(boost::format(format) % threshold % (n + 1));
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

void match(const DescriptorList& descriptors1,
           const DescriptorList& descriptors2,
           MatchList& matches,
           bool use_flann,
           double& time) {
  // Load descriptors into matrices for cv::DescriptorMatcher.
  cv::Mat mat1;
  cv::Mat mat2;
  listToMatrix(descriptors1, mat1);
  listToMatrix(descriptors2, mat2);

  // Find two nearest neighbours, match in each direction.
  std::vector<MatchResultList> forward_match_lists;
  std::vector<MatchResultList> reverse_match_lists;

  clock_t t = clock();

  // Construct matcher.
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (use_flann) {
    matcher = cv::DescriptorMatcher::create("FlannBased");
  } else {
    matcher = cv::DescriptorMatcher::create("BruteForce");
  }

  // Match all points in both directions.
  // Get top two matches as this is what we'll be doing in practice.
  matcher->knnMatch(mat1, mat2, forward_match_lists, 2);
  matcher->knnMatch(mat2, mat1, reverse_match_lists, 2);

  t = clock() - t;
  time = double(t) / CLOCKS_PER_SEC;

  // Keep only first match.
  MatchList forward_matches;
  MatchList reverse_matches;
  std::transform(forward_match_lists.begin(), forward_match_lists.end(),
      std::back_inserter(forward_matches), extractFirstMatch);
  std::transform(reverse_match_lists.begin(), reverse_match_lists.end(),
      std::back_inserter(reverse_matches), extractFirstMatch);

  // Now check consistency in both directions.
  std::remove_copy_if(forward_matches.begin(), forward_matches.end(),
      std::back_inserter(matches), IsInconsistent(reverse_matches));

  LOG(INFO) << forward_matches.size() << " forward matches";
  LOG(INFO) << reverse_matches.size() << " reverse matches";
  LOG(INFO) << matches.size() << " consistent matches";
}

int main(int argc, char** argv) {
  std::ostringstream usage;
  usage << "Evaluates effect of SIFT threshold on matches and speed." <<
    std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " descriptors1-format descriptors2-format base-threshold "
    "threshold-step num-thresholds" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  std::string descriptors_format1 = argv[1];
  std::string descriptors_format2 = argv[2];
  double base_threshold = boost::lexical_cast<double>(argv[3]);
  double threshold_step = boost::lexical_cast<double>(argv[4]);
  int num_thresholds = boost::lexical_cast<int>(argv[5]);

  // Really this should be farmed out MapReduce style.
  // No machine available at the moment though so why bother.
  for (int i = 0; i < num_thresholds; i += 1) {
    double threshold = base_threshold * std::pow(threshold_step, i);

    LOG(INFO) << "Threshold: " << threshold << std::endl;

    int t = 0;
    double mean_num_descriptors = 0;
    double mean_num_exhaustive_matches = 0;
    double mean_exhaustive_time = 0;
    double mean_num_flann_matches = 0;
    double mean_flann_time = 0;

    bool have_frames = true;

    while (have_frames) {
      // Build filenames.
      std::string descriptors_file1 = makeFilename(descriptors_format1,
          threshold, t);
      std::string descriptors_file2 = makeFilename(descriptors_format2,
          threshold, t);

      // Load descriptors.
      DescriptorList descriptors1;
      DescriptorList descriptors2;

      DescriptorReader descriptor_reader;
      VectorReader<Descriptor> reader(descriptor_reader);

      LOG(INFO) << "Loading descriptors from `" << descriptors_file1 << "'";
      have_frames = load(descriptors_file1, descriptors1, reader);
      if (!have_frames) {
        continue;
      }
      LOG(INFO) << "Loaded " << descriptors1.size() << " descriptors";

      LOG(INFO) << "Loading descriptors from `" << descriptors_file2 << "'";
      have_frames = load(descriptors_file2, descriptors2, reader);
      if (!have_frames) {
        continue;
      }
      LOG(INFO) << "Loaded " << descriptors2.size() << " descriptors";

      double exhaustive_time;
      double flann_time;
      MatchList exhaustive_matches;
      MatchList flann_matches;

      // Match the two lists of descriptors using brute force and FLANN.
      match(descriptors1, descriptors2, exhaustive_matches, false,
          exhaustive_time);
      match(descriptors1, descriptors2, flann_matches, true, flann_time);

      mean_num_descriptors += descriptors1.size();
      mean_num_descriptors += descriptors2.size();
      mean_num_exhaustive_matches += exhaustive_matches.size();
      mean_exhaustive_time += exhaustive_time;
      mean_num_flann_matches += flann_matches.size();
      mean_flann_time += flann_time;

      t += 1;
    }

    mean_num_descriptors /= 2 * t;
    mean_num_exhaustive_matches /= t;
    mean_exhaustive_time /= t;
    mean_num_flann_matches /= t;
    mean_flann_time /= t;

    std::cout << threshold << "\t";
    std::cout << mean_num_descriptors << "\t";
    std::cout << mean_num_exhaustive_matches << "\t";
    std::cout << mean_exhaustive_time << "\t";
    std::cout << mean_num_flann_matches << "\t";
    std::cout << mean_flann_time << std::endl;
  }

  return 0;
}
