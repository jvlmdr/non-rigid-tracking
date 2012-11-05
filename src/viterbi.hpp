#include <vector>
#include <deque>
#include <opencv2/core/core.hpp>

// Computes d(p) = min_q [ f(q) + g(p, q) ]
//
// p can take {0, ..., m - 1}
// q can take {0, ..., n - 1}
// f is length n
// g is m x n
void distanceTransform(const std::vector<double>& f,
                       const cv::Mat& g,
                       std::vector<double>& d,
                       std::vector<int>& arg);

// g[i] contains unary terms, h[i] contains binary terms
double solveViterbi(const std::deque<std::vector<double> >& g,
                    const std::vector<cv::Mat>& h,
                    std::vector<int>& x);

void quadraticDistanceTransform(const std::vector<double>& f,
                                std::vector<double>& d,
                                std::vector<int>& arg);

double solveViterbiQuadratic(const std::deque<std::vector<double> >& g,
                             std::vector<int>& x);

void quadraticDistanceTransform2D(const cv::Mat& f, cv::Mat& d, cv::Mat& arg);

double solveViterbiQuadratic2D(const std::vector<cv::Mat>& g,
                               std::vector<cv::Vec2i>& x);
