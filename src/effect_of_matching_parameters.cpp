#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "read_lines.hpp"
#include "vector_reader.hpp"
#include "default_reader.hpp"
#include "lexical_cast_parser.hpp"
#include "stats.hpp"

DEFINE_double(max_residual, 1.,
    "Maximum sum-of-squares pixel error to be an inlier.");

struct Stats {
  double num_matches;
  double num_correct;
  double mean_residual;
  double median_residual;
};

void writeStats(std::ostream& os, const Stats& stats) {
  os << stats.num_matches << "\t";
  os << stats.num_correct << "\t";
  os << stats.mean_residual << "\t";
  os << stats.median_residual;
}

bool fileExists(const std::string& filename) {
  return std::ifstream(filename.c_str()).good();
}

void reduceResidualsOverFrames(const std::string& residuals_format,
                               Stats& stats,
                               double max_residual) {
  int i = 0;
  bool finished = false;
  DefaultReader<double> reader;

  // Take mean over all frames.
  double num_matches = 0;
  double num_correct = 0;
  double mean_residual = 0;
  double median_residual = 0;

  while (!finished) {
    // Form file name.
    std::string residuals_file = boost::str(
        boost::format(residuals_format) % (i + 1));

    if (!fileExists(residuals_file)) {
      finished = true;
      continue;
    }

    // Load residuals.
    std::vector<double> residuals;
    bool ok = loadList(residuals_file, residuals, reader);
    CHECK(ok) << "Could not load residuals";

    // Count number of matches.
    num_matches += residuals.size();
    num_correct += countLessThanEqualTo(residuals, max_residual);
    mean_residual += computeMean(residuals);
    median_residual += computeMedian(residuals);

    i += 1;
  }

  stats.num_matches = num_matches / i;
  stats.num_correct = num_correct / i;
  stats.mean_residual = mean_residual / i;
  stats.median_residual = median_residual / i;
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Reduces results from feature matching with varying parameters." <<
      std::endl;
  usage << std::endl;
  usage << "Sample usage:" << std::endl;
  usage << argv[0] << " residuals-format contrast-thresholds"
      " distance-thresholds effect-of-contrast effect-of-distance" << std::endl;
  usage << std::endl;
  usage << "Parameters:" << std::endl;
  usage << "residuals-format -- Filename with two string parameters (%s). The"
      " first is for the contrast threshold, the second for distance"
      " threshold. Should also contain an escaped integer parameter (%%d) for"
      " frame number." << std::endl;
  usage << "contrast-thresholds -- File containing a contrast threshold per"
      " line." << std::endl;
  usage << "distance-thresholds -- File containing a distance threshold per"
      " line." << std::endl;

  google::SetUsageMessage(usage.str());
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc != 6) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string residuals_format_format = argv[1];
  std::string contrast_thresholds_file = argv[2];
  std::string distance_thresholds_file = argv[3];
  std::string contrast_data_file = argv[4];
  std::string distance_data_file = argv[5];

  bool ok;

  // Read contrast thresholds.
  std::vector<std::string> contrast_thresholds;
  ok = readLines(contrast_thresholds_file, contrast_thresholds);
  CHECK(ok) << "Could not read contrast thresholds";

  // Read distance thresholds.
  std::vector<std::string> distance_thresholds;
  ok = readLines(distance_thresholds_file, distance_thresholds);
  CHECK(ok) << "Could not read distance thresholds";

  int num_contrast_thresholds = contrast_thresholds.size();
  int num_distance_thresholds = distance_thresholds.size();

  std::vector<std::vector<Stats> > stats;

  // Initialize 2D vector.
  stats.assign(num_contrast_thresholds, std::vector<Stats>());
  for (int i = 0; i < num_contrast_thresholds; i += 1) {
    stats[i].assign(num_distance_thresholds, Stats());
  }

  // Iterate over contrast thresholds.
  for (int i = 0; i < num_contrast_thresholds; i += 1) {
    std::string contrast_threshold = contrast_thresholds[i];

    // Iterate over distance thresholds.
    for (int j = 0; j < num_distance_thresholds; j += 1) {
      std::string distance_threshold = distance_thresholds[j];

      LOG(INFO) << "(" << contrast_threshold << ", " << distance_threshold <<
        ")";

      // Build format for (contrast_threshold, distance_threshold).
      std::string residuals_format = boost::str(
          boost::format(residuals_format_format) % contrast_thresholds[i]
                                                 % distance_thresholds[j]);

      reduceResidualsOverFrames(residuals_format, stats[i][j],
          FLAGS_max_residual);
    }
  }


  // Write out grouped by distance.
  {
    std::ofstream ofs;
    ofs.open(contrast_data_file.c_str());
    CHECK(ofs.is_open()) << "Could not write to data file";
    for (int j = 0; j < num_distance_thresholds; j += 1) {
      for (int i = 0; i < num_contrast_thresholds; i += 1) {
        ofs << contrast_thresholds[i] << "\t";
        ofs << distance_thresholds[j] << "\t";
        writeStats(ofs, stats[i][j]);
        ofs << std::endl;
      }
      ofs << std::endl;
      ofs << std::endl;
    }
  }

  // Write out grouped by contrast.
  {
    std::ofstream ofs;
    ofs.open(distance_data_file.c_str());
    CHECK(ofs.is_open()) << "Could not write to data file";
    for (int i = 0; i < num_contrast_thresholds; i += 1) {
      for (int j = 0; j < num_distance_thresholds; j += 1) {
        ofs << contrast_thresholds[i] << "\t";
        ofs << distance_thresholds[j] << "\t";
        writeStats(ofs, stats[i][j]);
        ofs << std::endl;
      }
      ofs << std::endl;
      ofs << std::endl;
    }
  }

  return 0;
}
