#include <vector>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "match_result.hpp"
#include "optimal_triangulation.hpp"

#include "match_result_reader.hpp"
#include "image_point_reader.hpp"
#include "iterator_reader.hpp"
#include "matrix_reader.hpp"
#include "iterator_writer.hpp"
#include "match_result_writer.hpp"

DEFINE_double(max_residual, 2.,
    "Maximum deviation from epipolar line in pixels");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Removes the matches which do not fit rigid geometry." << std::endl;
  usage << std::endl;
  usage << argv[0] << " matches keypoints1 keypoints2 fund-mat rigid-matches" <<
      std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

struct MatchIsOutlier {
  const std::vector<cv::Point2d>* points1;
  const std::vector<cv::Point2d>* points2;
  const cv::Mat* F;
  double max_residual;

  bool operator()(const MatchResult& match) const {
    cv::Point2d point1 = (*points1)[match.index1];
    cv::Point2d point2 = (*points2)[match.index2];

    double residual = optimalTriangulation(point1, point2, *F);

    return (residual > max_residual);
  }

  MatchIsOutlier(const std::vector<cv::Point2d>& points1,
                 const std::vector<cv::Point2d>& points2,
                 const cv::Mat& F,
                 double max_residual)
      : points1(&points1),
        points2(&points2),
        F(&F),
        max_residual(max_residual) {}
};

int main(int argc, char** argv) {
  init(argc, argv);

  std::string matches_file = argv[1];
  std::string keypoints_file1 = argv[2];
  std::string keypoints_file2 = argv[3];
  std::string fund_mat_file = argv[4];
  std::string inliers_file = argv[5];

  bool ok;

  // Load matches.
  std::vector<MatchResult> matches;
  MatchResultReader match_reader;
  ok = loadList(matches_file, matches, match_reader);
  CHECK(ok) << "Could not load matches";

  // Load points.
  std::vector<cv::Point2d> points1;
  std::vector<cv::Point2d> points2;
  ImagePointReader<double> point_reader;
  ok = loadList(keypoints_file1, points1, point_reader);
  CHECK(ok) << "Could not load points";
  ok = loadList(keypoints_file2, points2, point_reader);
  CHECK(ok) << "Could not load points";

  // Load fundamental matrix.
  cv::Mat F;
  MatrixReader matrix_reader;
  ok = load(fund_mat_file, F, matrix_reader);
  CHECK(ok) << "Could not load fundamental matrix";

  std::vector<MatchResult> inliers;
  // Construct test.
  MatchIsOutlier match_is_outlier(points1, points2, F, FLAGS_max_residual);
  // Remove outliers.
  std::remove_copy_if(matches.begin(), matches.end(),
      std::back_inserter(inliers), match_is_outlier);

  int num_input = matches.size();
  int num_output = inliers.size();
  double fraction = static_cast<double>(num_output) / num_input;
  LOG(INFO) << "Kept " << num_output << " / " << num_input << " matches (" <<
      fraction << ")";

  MatchResultWriter match_writer;
  ok = saveList(inliers_file, inliers, match_writer);
  CHECK(ok) << "Could not save rigid matches to file";

  return 0;
}
