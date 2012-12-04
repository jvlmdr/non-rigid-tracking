#include <vector>
#include <deque>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include "viterbi.hpp"
#include "util.hpp"

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

TEST(SolveViterbiPartialQuadratic2D, VersusNaive) {
  int n = 17;
  int k_B = 13;
  int p = 7;
  int q = 11;

  int k_A = p * q;
  int k = k_A + k_B;

  // Initialize g_A randomly.
  std::vector<cv::Mat> g_A(n);
  for (int i = 0; i < n; i += 1) {
    g_A[i] = cv::Mat_<double>(p, q);
    cv::randn(g_A[i], 0, 1);
  }

  // Initialize g_B randomly.
  std::deque<std::vector<double> > g_B(n);
  for (int i = 0; i < n; i += 1) {
    g_B[i].assign(k_B, 0.);
    cv::randn(g_B[i], 0, 1);
  }

  // Initialize h_AB randomly.
  std::vector<cv::Mat> h_AB(n - 1);
  for (int i = 0; i < n - 1; i += 1) {
    int dims[] = { k_B, p, q };
    h_AB[i] = cv::Mat_<double>(3, dims);
    cv::randn(h_AB[i], 0, 1);
  }

  // Initialize h_BA randomly.
  std::vector<cv::Mat> h_BA(n - 1);
  for (int i = 0; i < n - 1; i += 1) {
    int dims[] = { p, q, k_B };
    h_BA[i] = cv::Mat_<double>(3, dims);
    cv::randn(h_BA[i], 0, 1);
  }

  // Initialize h_BB randomly.
  std::vector<cv::Mat> h_BB(n - 1);
  for (int i = 0; i < n - 1; i += 1) {
    h_BB[i] = cv::Mat_<double>(k_B, k_B);
    cv::randn(h_BB[i], 0, 1);
  }

  // Flatten out unary terms.
  std::deque<std::vector<double> > g(n);
  for (int i = 0; i < n; i += 1) {
    std::copy(g_A[i].begin<double>(), g_A[i].end<double>(),
        std::back_inserter(g[i]));
    std::copy(g_B[i].begin(), g_B[i].end(), std::back_inserter(g[i]));
  }

  // Construct quadratic distance matrix.
  cv::Mat h_AA;
  {
    int dims[] = { p, q, p, q };
    h_AA = cv::Mat_<double>(4, dims, -std::numeric_limits<double>::infinity());

    for (int i = 0; i < p; i += 1) {
      for (int j = 0; j < q; j += 1) {
        for (int u = 0; u < p; u += 1) {
          for (int v = 0; v < q; v += 1) {
            int index[] = { i, j, u, v };
            h_AA.at<double>(index) = sqr(i - u) + sqr(j - v);
          }
        }
      }
    }
  }
  {
    int dims[] = { k_A, k_A };
    h_AA = h_AA.reshape(1, 2, dims);
  }

  // Flatten and fuse pairwise terms.
  std::vector<cv::Mat> h(n - 1);
  for (int i = 0; i < n - 1; i += 1) {
    // Initialize to -inf so that errors are immediately obvious.
    h[i] = cv::Mat_<double>(k, k, -std::numeric_limits<double>::infinity());

    // A to A
    cv::Mat dst = h[i](cv::Range(0, k_A), cv::Range(0, k_A));
    h_AA.copyTo(dst);

    // B to A
    {
      int dims[] = { k_A, k_B };
      cv::Mat src = h_BA[i].reshape(1, 2, dims);
      dst = h[i](cv::Range(0, k_A), cv::Range(k_A, k));
      src.copyTo(dst);
    }

    // A to B
    {
      int dims[] = { k_B, k_A };
      cv::Mat src = h_AB[i].reshape(1, 2, dims);
      dst = h[i](cv::Range(k_A, k), cv::Range(0, k_A));
      src.copyTo(dst);
    }

    // B to B
    dst = h[i](cv::Range(k_A, k), cv::Range(k_A, k));
    h_BB[i].copyTo(dst);
  }

  // Solve naive dynamic program.
  std::vector<int> x_naive;
  double f_naive = solveViterbi(g, h, x_naive);

  // Solve with the partially accelerated distance transform.
  std::vector<MixedVariable> x;
  double f = solveViterbiPartialQuadratic2D(g_A, g_B, h_AB, h_BA, h_BB, x);

  // Check value.
  ASSERT_EQ(f_naive, f);

  // Check args.
  for (int i = 0; i < int(x_naive.size()); i += 1) {
    ASSERT_EQ(x_naive[i] < k_A, x[i].set == 0);

    if (x[i].set == 0) {
      ASSERT_EQ(cv::Vec2i(x_naive[i] / q, x_naive[i] % q), x[i].two_d);
    } else {
      ASSERT_EQ(x_naive[i] - k_A, x[i].one_d);
    }
  }
}

TEST(SolveViterbiSplitQuadratic2D, VersusNaive) {
  int length = 17;
  int m = 7;
  int n = 11;
  int p = 5;
  int q = 13;

  int k_A = m * n;
  int k_B = p * q;
  int k = k_A + k_B;

  // Initialize g_A randomly.
  std::vector<cv::Mat> g_A(length);
  for (int i = 0; i < length; i += 1) {
    g_A[i] = cv::Mat_<double>(m, n);
    cv::randn(g_A[i], 0, 1);
  }

  // Initialize g_B randomly.
  std::vector<cv::Mat> g_B(length);
  for (int i = 0; i < length; i += 1) {
    g_B[i] = cv::Mat_<double>(p, q);
    cv::randn(g_B[i], 0, 1);
  }

  // Flatten out unary terms.
  std::deque<std::vector<double> > g(length);
  for (int i = 0; i < length; i += 1) {
    std::copy(g_A[i].begin<double>(), g_A[i].end<double>(),
        std::back_inserter(g[i]));
    std::copy(g_B[i].begin<double>(), g_B[i].end<double>(),
        std::back_inserter(g[i]));
  }

  // Construct quadratic distance matrix for A to A.
  cv::Mat h_AA;
  {
    int dims[] = { m, n, m, n };
    h_AA = cv::Mat_<double>(4, dims, -std::numeric_limits<double>::infinity());

    for (int i = 0; i < m; i += 1) {
      for (int j = 0; j < n; j += 1) {
        for (int u = 0; u < m; u += 1) {
          for (int v = 0; v < n; v += 1) {
            int index[] = { i, j, u, v };
            h_AA.at<double>(index) = sqr(i - u) + sqr(j - v);
          }
        }
      }
    }
  }

  // Construct quadratic distance matrix for A to B.
  cv::Mat h_AB;
  {
    int dims[] = { p, q, m, n };
    h_AB = cv::Mat_<double>(4, dims, -std::numeric_limits<double>::infinity());

    for (int i = 0; i < p; i += 1) {
      for (int j = 0; j < q; j += 1) {
        for (int u = 0; u < m; u += 1) {
          for (int v = 0; v < n; v += 1) {
            int index[] = { i, j, u, v };
            h_AB.at<double>(index) = sqr(i - u) + sqr(j - v);
          }
        }
      }
    }
  }

  // Construct quadratic distance matrix for B to A.
  cv::Mat h_BA;
  {
    int dims[] = { m, n, p, q };
    h_BA = cv::Mat_<double>(4, dims, -std::numeric_limits<double>::infinity());

    for (int i = 0; i < m; i += 1) {
      for (int j = 0; j < n; j += 1) {
        for (int u = 0; u < p; u += 1) {
          for (int v = 0; v < q; v += 1) {
            int index[] = { i, j, u, v };
            h_BA.at<double>(index) = sqr(i - u) + sqr(j - v);
          }
        }
      }
    }
  }

  // Construct quadratic distance matrix for B to B.
  cv::Mat h_BB;
  {
    int dims[] = { p, q, p, q };
    h_BB = cv::Mat_<double>(4, dims, -std::numeric_limits<double>::infinity());

    for (int i = 0; i < p; i += 1) {
      for (int j = 0; j < q; j += 1) {
        for (int u = 0; u < p; u += 1) {
          for (int v = 0; v < q; v += 1) {
            int index[] = { i, j, u, v };
            h_BB.at<double>(index) = sqr(i - u) + sqr(j - v);
          }
        }
      }
    }
  }

  // Flatten and fuse pairwise terms.
  std::vector<cv::Mat> h(length - 1);
  for (int i = 0; i < length - 1; i += 1) {
    // Initialize to -inf so that errors are immediately obvious.
    h[i] = cv::Mat_<double>(k, k, -std::numeric_limits<double>::infinity());

    // A to A
    {
      int dims[] = { k_A, k_A };
      cv::Mat src = h_AA.reshape(1, 2, dims);
      cv::Mat dst = h[i](cv::Range(0, k_A), cv::Range(0, k_A));
      src.copyTo(dst);
    }

    // B to A
    {
      int dims[] = { k_A, k_B };
      cv::Mat src = h_BA.reshape(1, 2, dims);
      cv::Mat dst = h[i](cv::Range(0, k_A), cv::Range(k_A, k));
      src.copyTo(dst);
    }

    // A to B
    {
      int dims[] = { k_B, k_A };
      cv::Mat src = h_AB.reshape(1, 2, dims);
      cv::Mat dst = h[i](cv::Range(k_A, k), cv::Range(0, k_A));
      src.copyTo(dst);
    }

    // B to B
    {
      int dims[] = { k_B, k_B };
      cv::Mat src = h_BB.reshape(1, 2, dims);
      cv::Mat dst = h[i](cv::Range(k_A, k), cv::Range(k_A, k));
      src.copyTo(dst);
    }
  }

  // Solve naive dynamic program.
  std::vector<int> x_naive;
  double f_naive = solveViterbi(g, h, x_naive);

  // Solve with the partially accelerated distance transform.
  std::vector<SplitVariable> x;
  double f = solveViterbiSplitQuadratic2D(g_A, g_B, x);

  // Check value.
  ASSERT_EQ(f_naive, f);

  // Check args.
  for (int t = 0; t < int(x_naive.size()); t += 1) {
    ASSERT_EQ(x_naive[t] < k_A, x[t].set == 0);

    cv::Vec2i index;
    if (x[t].set == 0) {
      int i = x_naive[t];
      index = cv::Vec2i(i / n, i % n);
    } else {
      int i = x_naive[t] - k_A;
      index = cv::Vec2i(i / q, i % q);
    }
    ASSERT_EQ(index, x[t].index);
  }
}
