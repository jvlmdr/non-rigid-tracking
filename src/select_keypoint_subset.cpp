#include <string>
#include <cstdlib>
#include <sstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/scoped_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sift_position_reader.hpp"
#include "default_reader.hpp"
#include "iterator_reader.hpp"

#include "sift_position_writer.hpp"
#include "iterator_writer.hpp"

DEFINE_int32(radius, 5, "Base radius of features");
DEFINE_int32(line_thickness, 1, "Line thickness");

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Selects a subset of keypoints" << std::endl;
  usage << std::endl;
  usage << argv[0] << " keypoints indices subset" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 4) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  bool ok;

  init(argc, argv);
  std::string keypoints_file = argv[1];
  std::string indices_file = argv[2];
  std::string subset_file = argv[3];

  // Load keypoints.
  std::vector<SiftPosition> keypoints;
  SiftPositionReader feature_reader;
  ok = loadList(keypoints_file, keypoints, feature_reader);
  CHECK(ok) << "Could not load keypoints";
  int num_keypoints = keypoints.size();
  LOG(INFO) << "Loaded " << num_keypoints << " keypoints";

  // Load indices.
  std::vector<int> indices;
  DefaultReader<int> index_reader;
  ok = loadList(indices_file, indices, index_reader);
  CHECK(ok) << "Could not load indices";

  // Extract subset.
  std::vector<SiftPosition> subset;

  std::vector<int>::const_iterator index;
  for (index = indices.begin(); index != indices.end(); ++index) {
    subset.push_back(keypoints[*index]);
  }

  SiftPositionWriter index_writer;
  ok = saveList(subset_file, subset, index_writer);
  CHECK(ok) << "Could not save keypoint subset";
  LOG(INFO) << "Selected " << subset.size() << " / " << num_keypoints <<
      " keypoints";

  return 0;
}
