#include <vector>
#include <deque>
#include <opencv2/core/core.hpp>
#include "viterbi.hpp"
#include "gtest/gtest.h"
#include "glog/logging.h"

TEST(SolveViterbi, VersusExhaustive) {
  int n = 4;
  int k = 30;

  std::deque<std::vector<double> > g(n);
  for (int i = 0; i < n; i += 1) {
    std::vector<double> tmp(k);
    g[i].swap(tmp);
    cv::randn(g[i], 0, 1);
  }

  std::vector<cv::Mat> h(n - 1);
  for (int i = 0; i < n - 1; i += 1) {
    h[i] = cv::Mat_<double>(k, k);
    cv::randn(h[i], 0, 1);
  }

  std::vector<int> x(n);

  double f_brute = 0;
  std::vector<int> x_brute;

  for (x[0] = 0; x[0] < k; x[0] += 1) {
    for (x[1] = 0; x[1] < k; x[1] += 1) {
      for (x[2] = 0; x[2] < k; x[2] += 1) {
        for (x[3] = 0; x[3] < k; x[3] += 1) {
          double f = g[0][x[0]] + g[1][x[1]] + g[2][x[2]] + g[3][x[3]] +
              h[0].at<double>(x[1], x[0]) +
              h[1].at<double>(x[2], x[1]) +
              h[2].at<double>(x[3], x[2]);

          if (x_brute.empty() || f < f_brute) {
            f_brute = f;
            x_brute = x;
          }
        }
      }
    }
  }

  std::vector<int> x_viterbi;
  double f_viterbi = solveViterbi(g, h, x_viterbi);

  ASSERT_EQ(f_brute, f_viterbi);

  for (int i = 0; i < n; i += 1) {
    ASSERT_EQ(x_brute[i], x_viterbi[i]);
  }
}

TEST(SolveViterbiQuadratic, VersusExhaustive) {
  int n = 4;
  int k = 30;

  std::deque<std::vector<double> > g(n);
  for (int i = 0; i < n; i += 1) {
    std::vector<double> tmp(k);
    g[i].swap(tmp);
    cv::randn(g[i], 0, 1);
  }

  std::vector<int> x(n);

  double f_brute = 0;
  std::vector<int> x_brute;

  for (x[0] = 0; x[0] < k; x[0] += 1) {
    for (x[1] = 0; x[1] < k; x[1] += 1) {
      for (x[2] = 0; x[2] < k; x[2] += 1) {
        for (x[3] = 0; x[3] < k; x[3] += 1) {
          double f = g[0][x[0]] + g[1][x[1]] + g[2][x[2]] + g[3][x[3]] +
              (x[0] - x[1]) * (x[0] - x[1]) +
              (x[1] - x[2]) * (x[1] - x[2]) +
              (x[2] - x[3]) * (x[2] - x[3]);

          if (x_brute.empty() || f < f_brute) {
            f_brute = f;
            x_brute = x;
          }
        }
      }
    }
  }

  std::vector<int> x_viterbi;
  double f_viterbi = solveViterbiQuadratic(g, x_viterbi);

  ASSERT_EQ(f_brute, f_viterbi);

  for (int i = 0; i < n; i += 1) {
    ASSERT_EQ(x_brute[i], x_viterbi[i]);
  }
}

TEST(SolveViterbiQuadratic2D, VersusExhaustive) {
  int n = 4;
  int kx = 7;
  int ky = 11;

  std::vector<cv::Mat> g(n);
  for (int i = 0; i < n; i += 1) {
    g[i] = cv::Mat_<double>(ky, kx, 0.);
    cv::randn(g[i], 0, 1);
  }

  double f_brute = 0;
  std::vector<int> x_brute;
  std::vector<int> y_brute;

  std::vector<int> x(n);
  std::vector<int> y(n);

  for (x[0] = 0; x[0] < kx; x[0] += 1) {
    for (y[0] = 0; y[0] < ky; y[0] += 1) {
      for (x[1] = 0; x[1] < kx; x[1] += 1) {
        for (y[1] = 0; y[1] < ky; y[1] += 1) {
          for (x[2] = 0; x[2] < kx; x[2] += 1) {
            for (y[2] = 0; y[2] < ky; y[2] += 1) {
              for (x[3] = 0; x[3] < kx; x[3] += 1) {
                for (y[3] = 0; y[3] < ky; y[3] += 1) {
                  double f =
                      g[0].at<double>(y[0], x[0]) +
                      g[1].at<double>(y[1], x[1]) +
                      g[2].at<double>(y[2], x[2]) +
                      g[3].at<double>(y[3], x[3]) +
                      (x[0] - x[1]) * (x[0] - x[1]) +
                      (y[0] - y[1]) * (y[0] - y[1]) +
                      (x[1] - x[2]) * (x[1] - x[2]) +
                      (y[1] - y[2]) * (y[1] - y[2]) +
                      (x[2] - x[3]) * (x[2] - x[3]) +
                      (y[2] - y[3]) * (y[2] - y[3]);

                  if (x_brute.empty() || f < f_brute) {
                    f_brute = f;
                    x_brute = x;
                    y_brute = y;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  std::vector<cv::Vec2i> x_viterbi;
  double f_viterbi = solveViterbiQuadratic2D(g, x_viterbi);

  ASSERT_EQ(f_brute, f_viterbi);

  for (int i = 0; i < n; i += 1) {
    ASSERT_EQ(x_brute[i], x_viterbi[i][1]);
    ASSERT_EQ(y_brute[i], x_viterbi[i][0]);
  }
}

TEST(SolveViterbiQuadratic2D, VersusNaive) {
  int n = 256;
  int kx = 7;
  int ky = 11;
  int k = kx * ky;

  std::vector<cv::Mat> g_matrix(n);
  std::deque<std::vector<double> > g_vector(n);
  std::vector<cv::Mat> h(n - 1);

  for (int i = 0; i < n; i += 1) {
    // Set unary terms randomly.
    g_matrix[i] = cv::Mat_<double>(ky, kx, 0.);
    cv::randn(g_matrix[i], 0, 1);

    // Vectorize.
    g_vector[i] = std::vector<double>(k);
    for (int x = 0; x < kx; x += 1) {
      for (int y = 0; y < ky; y += 1) {
        int p = y + ky * x;
        g_vector[i][p] = g_matrix[i].at<double>(y, x);
      }
    }
  }

  for (int i = 0; i < n - 1; i += 1) {
    // Compute pairwise terms.
    h[i] = cv::Mat_<double>(k, k);

    for (int x = 0; x < kx; x += 1) {
      for (int y = 0; y < ky; y += 1) {
        int p = y + ky * x;

        for (int u = 0; u < kx; u += 1) {
          for (int v = 0; v < ky; v += 1) {
            int q = v + ky * u;
            h[i].at<double>(p, q) = (u - x) * (u - x) + (v - y) * (v - y);
          }
        }
      }
    }
  }

  std::vector<int> x_vector;
  double f_vector = solveViterbi(g_vector, h, x_vector);

  std::vector<cv::Vec2i> x_matrix;
  double f_matrix = solveViterbiQuadratic2D(g_matrix, x_matrix);

  ASSERT_EQ(f_vector, f_matrix);

  for (int i = 0; i < n; i += 1) {
    ASSERT_EQ(x_matrix[i][1], x_vector[i] / ky);
    ASSERT_EQ(x_matrix[i][0], x_vector[i] % ky);
  }
}
