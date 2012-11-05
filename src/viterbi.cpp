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

  for (int p = 0; p < n; p += 1) {
    double d_star = 0;
    int q_star = -1;

    for (int q = 0; q < m; q += 1) {
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
  cv::Mat d_i(f.rows, f.cols, cv::DataType<double>::type);
  cv::Mat arg_i(f.rows, f.cols, cv::DataType<int>::type);

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
      d_i.at<double>(i, j) = y[j];
      arg_i.at<int>(i, j) = x[j];
    }
  }

  d.create(f.rows, f.cols, cv::DataType<double>::type);
  arg.create(f.rows, f.cols, cv::DataType<cv::Vec2i>::type);

  // Distance transform each column.
  for (int j = 0; j < f.cols; j += 1) {
    // Copy column into vector.
    std::vector<double> g(f.rows);
    for (int i = 0; i < f.rows; i += 1) {
      g[i] = d_i.at<double>(i, j);
    }

    // Find distance transform.
    std::vector<double> y;
    std::vector<int> x;
    quadraticDistanceTransform(g, y, x);

    for (int i = 0; i < f.rows; i += 1) {
      d.at<double>(i, j) = y[i];
      arg.at<cv::Vec2i>(i, j) = cv::Vec2i(x[i], arg_i.at<double>(x[i], j));
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
  // Remember to convert from (x, y) back to (i, j).
  x[n - 1] = cv::Vec2i(point.y, point.x);

  // Trace solutions back.
  for (int i = n - 1; i > 0; i -= 1) {
    x[i - 1] = args[i - 1].at<cv::Vec2i>(x[i]);
  }

  return f_star;
}
