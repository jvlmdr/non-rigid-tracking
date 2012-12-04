#include "viterbi.hpp"
#include <glog/logging.h>

void distanceTransform(const std::vector<double>& f,
                       const cv::Mat& g,
                       std::vector<double>& d,
                       std::vector<int>& arg) {
  CHECK(g.type() == cv::DataType<double>::type);
  int n = f.size();
  int m = g.rows;
  CHECK(g.cols == n);

  d.assign(m, 0);
  arg.assign(m, -1);

  // From p to q.
  for (int p = 0; p < m; p += 1) {
    double d_star = 0;
    int q_star = -1;

    for (int q = 0; q < n; q += 1) {
      double d_pq = f[q] + g.at<double>(p, q);

      if (q == 0 || d_pq < d_star) {
        d_star = d_pq;
        q_star = q;
      }
    }

    d[p] = d_star;
    arg[p] = q_star;
  }
}

double solveViterbi(const std::deque<std::vector<double> >& g,
                    const std::vector<cv::Mat>& h,
                    std::vector<int>& x) {
  int n = g.size();
  std::deque<std::vector<int> > args(n - 1);

  std::vector<double> f = g[0];

  for (int i = 0; i < n - 1; i += 1) {
    // Compute distance transform for next variable.
    std::vector<double> d;
    distanceTransform(f, h[i], d, args[i]);

    int k = g[i + 1].size();
    f.assign(k, 0);
    for (int j = 0; j < k; j += 1) {
      f[j] = d[j] + g[i + 1][j];
    }
  }

  x.assign(n, -1);

  // Find minimum element in final table.
  int k = g[n - 1].size();
  double f_star = 0;
  int j_star = -1;

  for (int j = 0; j < k; j += 1) {
    if (j == 0 || f[j] < f_star) {
      f_star = f[j];
      j_star = j;
    }
  }

  x[n - 1] = j_star;

  // Trace solutions back.
  for (int i = n - 1; i > 0; i -= 1) {
    x[i - 1] = args[i - 1][x[i]];
  }

  return f_star;
}

////////////////////////////////////////////////////////////////////////////////

void quadraticDistanceTransform(const std::vector<double>& f,
                                std::vector<double>& d,
                                std::vector<int>& arg) {
  d.clear();
  arg.clear();
  int n = f.size();

  // Indices of parabolas in the lower envelope.
  std::vector<int> v;
  v.push_back(0);

  // Intersections of parabolas.
  std::vector<double> z;

  for (int q = 1; q < n; q += 1) {
    bool found = false;

    while (!found) {
      // Find intersection with last parabola.
      int v_k = v.back();
      double s = ((f[q] + q * q) - (f[v_k] + v_k * v_k)) / (2 * q - 2 *  v_k);

      if (z.empty()) {
        // The lower bound contains one parabola and no intersections.
        found = true;
      } else {
        if (s > z.back()) {
          // This intersection occurs after the last one.
          found = true;
        }
      }

      if (found) {
        z.push_back(s);
        v.push_back(q);
      } else {
        z.pop_back();
        v.pop_back();
      }
    }
  }

  // The current parabola is v[k], the current bound is z[k].
  int k = 0;
  int max_k = z.size();

  for (int q = 0; q < n; q += 1) {
    while (k < max_k && z[k] < q) {
      k += 1;
    }

    double y = (q - v[k]) * (q - v[k]) + f[v[k]];

    d.push_back(y);
    arg.push_back(v[k]);
  }
}

double solveViterbiQuadratic(const std::deque<std::vector<double> >& g,
                             std::vector<int>& x) {
  int n = g.size();
  std::deque<std::vector<int> > args(n - 1);

  std::vector<double> f = g[0];

  for (int i = 0; i < n - 1; i += 1) {
    // Compute distance transform for next variable.
    std::vector<double> d;
    quadraticDistanceTransform(f, d, args[i]);

    int k = g[i + 1].size();
    f.assign(k, 0);
    for (int j = 0; j < k; j += 1) {
      f[j] = d[j] + g[i + 1][j];
    }
  }

  x.assign(n, -1);

  // Find minimum element in final table.
  int k = g[n - 1].size();
  double f_star = 0;
  int j_star = -1;

  for (int j = 0; j < k; j += 1) {
    if (j == 0 || f[j] < f_star) {
      f_star = f[j];
      j_star = j;
    }
  }

  x[n - 1] = j_star;

  // Trace solutions back.
  for (int i = n - 1; i > 0; i -= 1) {
    x[i - 1] = args[i - 1][x[i]];
  }

  return f_star;
}

////////////////////////////////////////////////////////////////////////////////

void quadraticDistanceTransform2D(const cv::Mat& f, cv::Mat& d, cv::Mat& arg) {
  CHECK(f.type() == cv::DataType<double>::type);

  // Distance transform for j given i.
  d.create(f.rows, f.cols, cv::DataType<double>::type);
  cv::Mat j_star(f.rows, f.cols, cv::DataType<int>::type);

  // Distance transform each row.
  for (int i = 0; i < f.rows; i += 1) {
    // Extract row.
    std::vector<double> g(f.cols);
    for (int j = 0; j < f.cols; j += 1) {
      g[j] = f.at<double>(i, j);
    }

    // Find distance transform.
    std::vector<double> y;
    std::vector<int> x;
    quadraticDistanceTransform(g, y, x);

    for (int j = 0; j < f.cols; j += 1) {
      d.at<double>(i, j) = y[j];
      j_star.at<int>(i, j) = x[j];
    }
  }

  cv::Mat i_star(f.rows, f.cols, cv::DataType<int>::type);

  // Distance transform each column.
  for (int j = 0; j < f.cols; j += 1) {
    // Copy column into vector.
    std::vector<double> g(f.rows);
    for (int i = 0; i < f.rows; i += 1) {
      g[i] = d.at<double>(i, j);
    }

    // Find distance transform.
    std::vector<double> y;
    std::vector<int> x;
    quadraticDistanceTransform(g, y, x);

    for (int i = 0; i < f.rows; i += 1) {
      d.at<double>(i, j) = y[i];
      i_star.at<int>(i, j) = x[i];
    }
  }

  arg.create(f.rows, f.cols, cv::DataType<cv::Vec2i>::type);

  for (int i = 0; i < f.rows; i += 1) {
    for (int j = 0; j < f.cols; j += 1) {
      int i_dash = i_star.at<int>(i, j);
      int j_dash = j_star.at<int>(i_dash, j);
      arg.at<cv::Vec2i>(i, j) = cv::Vec2i(i_dash, j_dash);
    }
  }
}

double solveViterbiQuadratic2D(const std::vector<cv::Mat>& g,
                               std::vector<cv::Vec2i>& x) {
  CHECK(!g.empty());

  int n = g.size();
  cv::Size size = g.front().size();

  cv::Mat f = g[0];
  std::vector<cv::Mat> args(n - 1);

  for (int i = 0; i < n - 1; i += 1) {
    // Compute distance transform for next variable.
    cv::Mat d;
    quadraticDistanceTransform2D(f, d, args[i]);
    f = d + g[i + 1];
  }

  // Find arg in final table.
  x.assign(n, cv::Vec2i::all(-1));
  double f_star;
  cv::Point point;
  cv::minMaxLoc(f, &f_star, NULL, &point, NULL);
  // Convert from (x, y) back to (i, j).
  x[n - 1] = cv::Vec2i(point.y, point.x);

  // Trace solutions back.
  for (int i = n - 1; i > 0; i -= 1) {
    x[i - 1] = args[i - 1].at<cv::Vec2i>(x[i]);
  }

  return f_star;
}

////////////////////////////////////////////////////////////////////////////////

MixedVariable::MixedVariable() : set(-1), two_d(-1, -1), one_d(-1) {}

// f(u, v) contains the sampled 2D function.
// g(i, u, v) contains the cost of transitioning from (u, v) to i.
//
// arg(i) contains the minimizer (u*, v*)
void distanceTransform2D1D(const cv::Mat& f,
                           const cv::Mat& g,
                           std::vector<double>& d,
                           std::vector<cv::Vec2i>& arg) {
  CHECK(g.type() == cv::DataType<double>::type);
  CHECK(g.dims == 3);

  int p = f.rows;
  int q = f.cols;

  int m = g.size[0];
  CHECK(g.size[1] == p);
  CHECK(g.size[2] == q);

  // Flatten from 3D tensor to matrix.
  // TODO: Avoid copying?
  std::vector<double> vec_f;
  std::copy(f.begin<double>(), f.end<double>(), std::back_inserter(vec_f));
  int dims[] = { m, p * q };
  cv::Mat vec_g = g.reshape(1, 2, dims);
  std::vector<int> index;
  distanceTransform(vec_f, vec_g, d, index);

  // Convert from 1D index to 2D index.
  arg.assign(m, cv::Vec2i(-1, -1));
  for (int i = 0; i < m; i += 1) {
    // Matrix is p x q. Un-flatten indices.
    arg[i] = cv::Vec2i(index[i] / q, index[i] % q);
  }
}

// f[i] contains the sampled 1D function.
// g(u, v, i) contains the cost of transitioning from i to (u, v).
//
// arg(u, v) contains the minimizer i*
void distanceTransform1D2D(const std::vector<double>& f,
                           const cv::Mat& g,
                           cv::Mat& d,
                           cv::Mat& arg) {
  CHECK(g.type() == cv::DataType<double>::type);
  CHECK(g.dims == 3);

  int n = f.size();
  int p = g.size[0];
  int q = g.size[1];
  CHECK(g.size[2] == n);
  int m = p * q;

  // Flatten from 3D tensor to matrix.
  int dims[] = { m, n };
  cv::Mat vec_g = g.reshape(1, 2, dims);

  std::vector<double> vec_d;
  std::vector<int> vec_arg;
  distanceTransform(f, vec_g, vec_d, vec_arg);

  // Pop out from matrix to tensor.
  // TODO: Avoid copying?
  d = cv::Mat(vec_d, true).reshape(1, p);
  arg = cv::Mat(vec_arg, true).reshape(1, p);
}

double solveViterbiPartialQuadratic2D(
    const std::vector<cv::Mat>& g_A,
    const std::deque<std::vector<double> >& g_B,
    const std::vector<cv::Mat>& h_AB,
    const std::vector<cv::Mat>& h_BA,
    const std::vector<cv::Mat>& h_BB,
    std::vector<MixedVariable>& x) {
  // Check sequences of unary terms have same length.
  int n = g_A.size();
  CHECK(g_B.size() == n);
  // Check binary terms are appropriate length.
  CHECK(h_BB.size() == n - 1);
  CHECK(h_AB.size() == n - 1);
  CHECK(h_BA.size() == n - 1);

  // Initialize partial solutions to unary term.
  cv::Mat f_A = g_A[0];
  std::vector<double> f_B = g_B[0];

  // Minimizer of each update.
  std::deque<std::deque<std::vector<MixedVariable> > > args_A;
  std::deque<std::vector<MixedVariable> > args_B;

  for (int i = 0; i < n - 1; i += 1) {
    // Compute distance transform.
    cv::Mat d_AA;
    cv::Mat args_AA;
    quadraticDistanceTransform2D(f_A, d_AA, args_AA);

    std::vector<double> d_AB;
    std::vector<cv::Vec2i> args_AB;
    distanceTransform2D1D(f_A, h_AB[i], d_AB, args_AB);

    cv::Mat d_BA;
    cv::Mat args_BA;
    distanceTransform1D2D(f_B, h_BA[i], d_BA, args_BA);

    std::vector<double> d_BB;
    std::vector<int> args_BB;
    distanceTransform(f_B, h_BB[i], d_BB, args_BB);

    // Take min over the two sets.

    // Dimensions of distance transform component.
    int p = g_A[i].rows;
    int q = g_A[i].cols;

    if (g_A[i].empty() || g_A[i + 1].empty()) {
      p = 0;
      q = 0;
    } else {
      CHECK(g_A[i].rows == g_A[i + 1].rows);
      CHECK(g_A[i].cols == g_A[i + 1].cols);
    }

    cv::Mat d_A = cv::Mat_<double>(p, q);
    std::deque<std::vector<MixedVariable> > args_A_i(p);
    for (int u = 0; u < p; u += 1) {
      args_A_i[u].assign(q, MixedVariable());
    }

    for (int u = 0; u < p; u += 1) {
      for (int v = 0; v < q; v += 1) {
        if (d_BA.at<double>(u, v) < d_AA.at<double>(u, v)) {
          d_A.at<double>(u, v) = d_BA.at<double>(u, v);
          args_A_i[u][v].set = 1;
          args_A_i[u][v].one_d = args_BA.at<int>(u, v);
        } else {
          d_A.at<double>(u, v) = d_AA.at<double>(u, v);
          args_A_i[u][v].set = 0;
          args_A_i[u][v].two_d = args_AA.at<cv::Vec2i>(u, v);
        }
      }
    }

    int k_B = g_B[i + 1].size();
    std::vector<double> d_B(k_B);
    std::vector<MixedVariable> args_B_i(k_B);
    for (int u = 0; u < k_B; u += 1) {
      if (d_BB[u] < d_AB[u]) {
        d_B[u] = d_BB[u];
        args_B_i[u].set = 1;
        args_B_i[u].one_d = args_BB[u];
      } else {
        d_B[u] = d_AB[u];
        args_B_i[u].set = 0;
        args_B_i[u].two_d = args_AB[u];
      }
    }

    args_A.push_back(args_A_i);
    args_B.push_back(args_B_i);

    // Add unary terms to result of distance transform.
    f_A = d_A + g_A[i + 1];
    f_B.assign(k_B, 0);
    for (int u = 0; u < k_B; u += 1) {
      f_B[u] = d_B[u] + g_B[i + 1][u];
    }
  }

  x.assign(n, MixedVariable());
  double f_star;

  // Find minimum element in final table.
  {
    double f_star_A = 0;
    cv::Point point;
    cv::minMaxLoc(f_A, &f_star_A, NULL, &point, NULL);
    if (point.x == -1 && point.y == -1) {
      // minMaxLoc does not work with infinities.
      point = cv::Point(0, 0);
      f_star_A = std::numeric_limits<double>::infinity();
    }
    cv::Vec2i x_star_A(point.y, point.x);

    double f_star_B = 0;
    int x_star_B = -1;
    int k_B = g_B[n - 1].size();
    for (int u = 0; u < k_B; u += 1) {
      if (u == 0 || f_B[u] < f_star_B) {
        f_star_B = f_B[u];
        x_star_B = u;
      }
    }

    if (f_star_B < f_star_A) {
      f_star = f_star_B;
      x[n - 1].set = 1;
      x[n - 1].one_d = x_star_B;
    } else {
      f_star = f_star_A;
      x[n - 1].set = 0;
      x[n - 1].two_d = x_star_A;
    }
  }

  // Trace solutions back.
  for (int i = n - 1; i > 0; i -= 1) {
    if (x[i].set == 0) {
      cv::Vec2i index = x[i].two_d;
      x[i - 1] = args_A[i - 1][index[0]][index[1]];
    } else {
      x[i - 1] = args_B[i - 1][x[i].one_d];
    }
  }

  return f_star;
}

////////////////////////////////////////////////////////////////////////////////

SplitVariable::SplitVariable() : set(-1), index(-1, -1) {}

// f(u, v) contains the sampled 2D function.
// g(i, j, u, v) contains the cost of transitioning from (u, v) to (i, j).
//
// arg(i, j) contains the minimizer (u*, v*)
void distanceTransform2D(const cv::Mat& f,
                         const cv::Mat& g,
                         cv::Mat& d,
                         cv::Mat& arg) {
  CHECK(g.type() == cv::DataType<double>::type);
  CHECK(g.dims == 4);

  int p = f.rows;
  int q = f.cols;

  int m = g.size[0];
  int n = g.size[1];
  CHECK(g.size[2] == p);
  CHECK(g.size[3] == q);

  // Flatten from 4D tensor to matrix.
  // TODO: Avoid copying?
  std::vector<double> vec_f;
  std::copy(f.begin<double>(), f.end<double>(), std::back_inserter(vec_f));
  int dims[] = { m * n, p * q };
  cv::Mat vec_g = g.reshape(1, 2, dims);
  std::vector<double> vec_d;
  std::vector<int> index;
  distanceTransform(vec_f, vec_g, vec_d, index);

  // Convert from 1D index to 2D index.
  std::vector<cv::Vec2i> vec_arg(m * n, cv::Vec2i(-1, -1));
  for (int i = 0; i < m * n; i += 1) {
    // Matrix is p x q. Un-flatten indices.
    vec_arg[i] = cv::Vec2i(index[i] / q, index[i] % q);
  }

  // Pop out from matrix to tensor.
  // TODO: Avoid copying?
  d = cv::Mat(vec_d, true).reshape(1, m);
  arg = cv::Mat(vec_arg, true).reshape(1, m);
}

double solveViterbiSplitQuadratic2D(
    const std::vector<cv::Mat>& g_A,
    const std::vector<cv::Mat>& g_B,
    std::vector<SplitVariable>& x) {
  // Check sequences of unary terms have same length.
  int length = g_A.size();
  CHECK(g_B.size() == length);

  // Initialize partial solutions to unary term.
  cv::Mat f_A = g_A[0];
  cv::Mat f_B = g_B[0];

  // Minimizer of each update.
  std::deque<std::deque<std::vector<SplitVariable> > > args_A;
  std::deque<std::deque<std::vector<SplitVariable> > > args_B;

  for (int t = 0; t < length - 1; t += 1) {
    // Compute distance transform.
    cv::Mat d_AA;
    cv::Mat args_AA;
    quadraticDistanceTransform2D(f_A, d_AA, args_AA);

    cv::Mat d_AB;
    cv::Mat args_AB;
    quadraticDistanceTransform2D(f_A, d_AB, args_AB);

    cv::Mat d_BA;
    cv::Mat args_BA;
    quadraticDistanceTransform2D(f_B, d_BA, args_BA);

    cv::Mat d_BB;
    cv::Mat args_BB;
    quadraticDistanceTransform2D(f_B, d_BB, args_BB);

    // Take min transitioning from either set to A.
    int m = g_A[t].rows;
    int n = g_A[t].cols;
    cv::Mat d_A = cv::Mat_<double>(m, n);

    std::deque<std::vector<SplitVariable> > args_A_t(m);
    for (int u = 0; u < m; u += 1) {
      args_A_t[u].assign(n, SplitVariable());
    }

    for (int u = 0; u < m; u += 1) {
      for (int v = 0; v < n; v += 1) {
        if (d_AA.at<double>(u, v) <= d_BA.at<double>(u, v)) {
          d_A.at<double>(u, v) = d_AA.at<double>(u, v);
          args_A_t[u][v].set = 0;
          args_A_t[u][v].index = args_AA.at<cv::Vec2i>(u, v);
        } else {
          d_A.at<double>(u, v) = d_BA.at<double>(u, v);
          args_A_t[u][v].set = 1;
          args_A_t[u][v].index = args_BA.at<cv::Vec2i>(u, v);
        }
      }
    }

    // Take min transitioning from either set to B.
    m = g_B[t].rows;
    n = g_B[t].cols;
    cv::Mat d_B = cv::Mat_<double>(m, n);

    std::deque<std::vector<SplitVariable> > args_B_t(m);
    for (int u = 0; u < m; u += 1) {
      args_B_t[u].assign(n, SplitVariable());
    }

    for (int u = 0; u < m; u += 1) {
      for (int v = 0; v < n; v += 1) {
        if (d_AB.at<double>(u, v) <= d_BB.at<double>(u, v)) {
          d_B.at<double>(u, v) = d_AB.at<double>(u, v);
          args_B_t[u][v].set = 0;
          args_B_t[u][v].index = args_AB.at<cv::Vec2i>(u, v);
        } else {
          d_B.at<double>(u, v) = d_BB.at<double>(u, v);
          args_B_t[u][v].set = 1;
          args_B_t[u][v].index = args_BB.at<cv::Vec2i>(u, v);
        }
      }
    }

    args_A.push_back(args_A_t);
    args_B.push_back(args_B_t);

    // Add unary terms to result of distance transform.
    f_A = d_A + g_A[t + 1];
    f_B = d_B + g_B[t + 1];
  }

  x.assign(length, SplitVariable());
  double f_star;

  // Find minimum element in final table.
  double f_star_A = 0;
  cv::Point point;
  cv::minMaxLoc(f_A, &f_star_A, NULL, &point, NULL);
  if (point.x == -1 && point.y == -1) {
    // minMaxLoc does not work with +inf.
    f_star_A = std::numeric_limits<double>::infinity();
    point = cv::Point(0, 0);
  }
  cv::Vec2i x_star_A(point.y, point.x);

  double f_star_B = 0;
  cv::minMaxLoc(f_B, &f_star_B, NULL, &point, NULL);
  if (point.x == -1 && point.y == -1) {
    // minMaxLoc does not work with +inf.
    f_star_A = std::numeric_limits<double>::infinity();
    point = cv::Point(0, 0);
  }
  cv::Vec2i x_star_B(point.y, point.x);

  if (f_star_A <= f_star_B) {
    f_star = f_star_A;
    x[length - 1].set = 0;
    x[length - 1].index = x_star_A;
  } else {
    f_star = f_star_B;
    x[length - 1].set = 1;
    x[length - 1].index = x_star_B;
  }

  // Trace solutions back.
  for (int t = length - 1; t > 0; t -= 1) {
    cv::Vec2i index = x[t].index;
    if (x[t].set == 0) {
      x[t - 1] = args_A[t - 1][index[0]][index[1]];
    } else {
      x[t - 1] = args_B[t - 1][index[0]][index[1]];
    }
  }

  return f_star;
}
