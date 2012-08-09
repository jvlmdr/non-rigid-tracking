#include <vector>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "match.hpp"
#include "match_reader.hpp"
#include "vector_reader.hpp"
#include "matrix_reader.hpp"
#include "optimal_triangulation.hpp"

typedef std::vector<Match> MatchList;

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Evaluates the quality of each of a set of matches." << std::endl;
  usage << "The quality of a match is the distance of each point from its"
    " match's epipolar line." << std::endl;
  usage << std::endl;
  usage << argv[0] << " matches keypoints1 keypoints2 fund-mat" << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
}

int main(int argc, char** argv) {
  init(argc, argv);

  // Read required parameters.
  if (argc != 5) {
    google::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  std::string matches_file = argv[1];
  std::string keypoints_file1 = argv[2];
  std::string keypoints_file2 = argv[3];
  std::string fund_mat_file = argv[4];

  bool ok;

  // Load matches.
  std::vector<Match> matches;
  MatchReader match_reader;
  VectorReader<Match> match_list_reader(match_reader);
  ok = load(matches_file, matches, match_list_reader);
  CHECK(ok) << "Could not load matches";

  // Load points.
  std::vector<cv::Point2d> points1;
  std::vector<cv::Point2d> points2;

  // Load fundamental matrix.
  cv::Mat F;
  MatrixReader matrix_reader;
  ok = load(fund_mat_file, F, matrix_reader);
  CHECK(ok) << "Could not load fundamental matrix";

  // Place where all the scores will go.
  std::vector<double> residuals;

  for (MatchList::const_iterator match = matches.begin();
       match != matches.end();
       ++match) {
    // Extract pair of points.
    cv::Point2d point1 = points1[match->first];
    cv::Point2d point2 = points2[match->second];

    // Perform optimal 2-view triangulation and record residual.
    double residual = optimalTriangulation(point1, point2, F);

    // Add score to list.
    residuals.push_back(residual);
  }

  // Write out residuals.

  return 0;
}
