#include <vector>
#include <deque>
#include <opencv2/core/core.hpp>

// Computes d(p) = min_q [ f(q) + g(p, q) ]
//
// p can take {0, ..., m - 1}
// q can take {0, ..., n - 1}
// f is length n
// g is m x n
//
// Finishes in O(mn) time
void distanceTransform(const std::vector<double>& f,
                       const cv::Mat& g,
                       std::vector<double>& d,
                       std::vector<int>& arg);

// g[i] contains unary terms, h[i] contains binary terms
double solveViterbi(const std::deque<std::vector<double> >& g,
                    const std::vector<cv::Mat>& h,
                    std::vector<int>& x);

////////////////////////////////////////////////////////////////////////////////

// Objectives with quadratic pairwise costs can be distance-transformed.

void quadraticDistanceTransform(const std::vector<double>& f,
                                std::vector<double>& d,
                                std::vector<int>& arg);

double solveViterbiQuadratic(const std::deque<std::vector<double> >& g,
                             std::vector<int>& x);

void quadraticDistanceTransform2D(const cv::Mat& f, cv::Mat& d, cv::Mat& arg);

double solveViterbiQuadratic2D(const std::vector<cv::Mat>& g,
                               std::vector<cv::Vec2i>& x);

////////////////////////////////////////////////////////////////////////////////

// Objectives where a subset of the variables can be distance-transformed.

struct MixedVariable {
  // If set == 0, then use two_d index. Otherwise, use one_d index.
  int set;
  cv::Vec2i two_d;
  int one_d;

  MixedVariable();
};

// The objective is split into regular (grid-aligned) and general arguments.
double solveViterbiPartialQuadratic2D(
    const std::vector<cv::Mat>& g_A,
    const std::deque<std::vector<double> >& g_B,
    const std::vector<cv::Mat>& h_AB,
    const std::vector<cv::Mat>& h_BA,
    const std::vector<cv::Mat>& h_BB,
    std::vector<MixedVariable>& x);
