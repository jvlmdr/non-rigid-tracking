#include <string>
#include <queue>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "track_list.hpp"
#include "track.hpp"
#include "viterbi.hpp"
#include "util.hpp"

#include "read_image.hpp"

DEFINE_int32(width, 1280, "Screen width");
DEFINE_int32(height, 800, "Screen width");

DEFINE_double(lambda, 1., "Temporal regularization");
DEFINE_double(rho, 1., "ADMM parameter");
DEFINE_double(rho_step, 1., "Rate to grow rho");

const int RADIUS = 5;
const int DIAMETER = 2 * RADIUS + 1;

////////////////////////////////////////////////////////////////////////////////

void findBestResponse(const cv::Point2d& z,
                      const cv::Point2d& u,
                      const cv::Mat& response,
                      double rho,
                      cv::Point2d& x) {
  cv::Size size = response.size();

  cv::Point2d v = z - u;

  // Compute distance from v.
  cv::Mat distance = cv::Mat_<double>(size);
  for (int i = 0; i < size.height; i += 1) {
    for (int j = 0; j < size.width; j += 1) {
      distance.at<double>(i, j) = sqr(v.x - j) + sqr(v.y - i);
    }
  }

  // Add distance to response.
  cv::Mat cost = response + rho / 2. * distance;

  // Find minimum.
  double min;
  cv::Point loc;
  cv::minMaxLoc(cost, &min, NULL, &loc, NULL);
  cv::Point2d p = loc;

  // Vector from p to v.
  cv::Point2d r = v - p;

  // Restrict to half-pixel trust region.
  double d = cv::norm(r);
  if (d > 0.5) {
    x = p + 0.5 / d * r;
  } else {
    x = v;
  }
}

// x is nx1, u is nx1, z is nx1
void findSmoothestPath(const cv::Mat& x,
                       const cv::Mat& u,
                       double rho,
                       double lambda,
                       cv::Mat& z) {
  int n = x.total();

  // Set up big linear system.
  cv::Mat D = cv::Mat_<double>(n - 1, n, 0.);
  for (int t = 0; t < n; t += 1) {
    D.at<double>(t, t) = 1;
    D.at<double>(t, t + 1) = -1;
  }

  cv::Mat I = cv::Mat_<double>::eye(n, n);
  cv::Mat A = I + 2. * lambda / rho * D.t() * D;
  cv::Mat v = x + u;

  z = A.inv() * v;
}

////////////////////////////////////////////////////////////////////////////////

std::string makeFrameFilename(const std::string& format, int time) {
  return boost::str(boost::format(format) % (time + 1));
}

void init(int& argc, char**& argv) {
  std::ostringstream usage;
  usage << "Linear time offline tracking" << std::endl;
  usage << std::endl;
  usage << argv[0] << " image-format num-frames" << std::endl;
  google::SetUsageMessage(usage.str());

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc != 3) {
    google::ShowUsageWithFlags(argv[0]);
    std::exit(1);
  }
}

bool tooBig(const cv::Size& image, const cv::Size& screen) {
  return image.width > screen.width || image.height > screen.height;
}

void loadImages(const std::string& format,
                int num_frames,
                const cv::Size& screen,
                std::vector<cv::Mat>& images) {
  images.clear();
  int downsample = 0;

  for (int t = 0; t < num_frames; t += 1) {
    cv::Mat image;
    std::string file = makeFrameFilename(format, t);
    bool ok = readColorImage(file, image);
    CHECK(ok) << "Could not load image";

    if (t == 0) {
      while (tooBig(image.size(), screen)) {
        cv::pyrDown(image, image);
        downsample += 1;
      }
    } else {
      for (int i = 0; i < downsample; i += 1) {
        cv::pyrDown(image, image);
      }
    }

    images.push_back(image);
  }
}

struct State {
  bool clicked;
  cv::Point2d point;
};

void onMouse(int event, int x, int y, int, void* tag) {
  State& state = *static_cast<State*>(tag);

  if (!state.clicked) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      state.point = cv::Point(x, y);
      state.clicked = true;
    }
  }
}

int main(int argc, char** argv) {
  init(argc, argv);

  std::string image_format = argv[1];
  int n = boost::lexical_cast<int>(argv[2]);
  CHECK(n > 0) << "Need at least one frame, two would be great";
  cv::Size screen_size(FLAGS_width, FLAGS_height);
  double rho = FLAGS_rho;
  double lambda = FLAGS_lambda;

  // Pre-load images.
  LOG(INFO) << "Loading images...";
  std::vector<cv::Mat> color_images;
  loadImages(image_format, n, screen_size, color_images);

  // Convert to grayscale.
  LOG(INFO) << "Loaded " << color_images.size() << " images";
  std::vector<cv::Mat> gray_images;
  std::vector<cv::Mat>::const_iterator color_image;
  for (color_image = color_images.begin();
       color_image != color_images.end();
       ++color_image) {
    cv::Mat gray_image;
    cv::cvtColor(*color_image, gray_image, CV_BGR2GRAY);
    gray_images.push_back(gray_image);
  }

  TrackList<cv::Point2d> track;

  bool exit = false;
  State state;
  state.clicked = false;

  cv::namedWindow("video");
  cv::setMouseCallback("video", onMouse, &state);

  // Display the image and ask the user to click a point.
  cv::imshow("video", color_images[0]);

  while (!state.clicked) {
    cv::waitKey(10);
  }

  // Extract a patch for cross-correlation.
  cv::Mat original;
  {
    cv::Point offset(state.point.x - RADIUS, state.point.y - RADIUS);
    cv::Rect region(offset, cv::Size(DIAMETER, DIAMETER));
    original = gray_images.front()(region);
  }

  // Match the template to every image.
  std::vector<cv::Mat> responses;
  LOG(INFO) << "Performing cross-correlation...";
  std::vector<cv::Mat>::const_iterator image;
  for (image = gray_images.begin(); image != gray_images.end(); ++image) {
    // Evaluate response to template.
    cv::Mat response;
    cv::matchTemplate(*image, original, response, cv::TM_CCORR_NORMED);
    // Convert to 64-bit.
    response = cv::Mat_<double>(response);
    response = -1. * response;
    // Add to list.
    responses.push_back(response);
  }

  // Solve dynamic program.
  cv::Mat x_star = cv::Mat_<double>(n, 2, 0.);
  LOG(INFO) << "Solving dynamic program...";
  {
    std::vector<cv::Mat> scaled;
    std::vector<cv::Mat>::const_iterator response;
    for (response = responses.begin();
         response != responses.end();
         ++response) {
      scaled.push_back(cv::Mat(1. / lambda * (*response)));
    }

    std::vector<cv::Vec2i> x;
    solveViterbiQuadratic2D(scaled, x);

    std::vector<cv::Vec2i>::const_iterator x_t;
    int t = 0;
    for (x_t = x.begin(); x_t != x.end(); ++x_t) {
      x_star.at<double>(t, 0) = (*x_t)[1];
      x_star.at<double>(t, 1) = (*x_t)[0];
      t += 1;
    }
  }

  LOG(INFO) << "Solving ADMM...";

  // Guess Lagrange multipliers to all be zero.
  cv::Mat x = cv::Mat_<double>(n, 2, 0.);
  // Initialize track positions to origin.
  cv::Mat z = cv::Mat_<double>(n, 2, 0.);
  // Smoothest path will also be origin.
  cv::Mat u = cv::Mat_<double>(n, 2, 0.);

  while (!exit) {
    LOG(INFO) << "rho => " << rho;

    // Solve first sub-problem.
    for (int t = 0; t < n; t += 1) {
      cv::Point2d z_t(z.at<double>(t, 0), z.at<double>(t, 1));
      cv::Point2d u_t(u.at<double>(t, 0), u.at<double>(t, 1));

      cv::Point2d x_t;
      findBestResponse(z_t, u_t, responses[t], rho, x_t);

      x.at<double>(t, 0) = x_t.x;
      x.at<double>(t, 1) = x_t.y;
    }
    //LOG(INFO) << "x-update: norm(x - z) => " << cv::norm(x - z) / n;

    cv::Mat z_old = z.clone();

    // Solve second sub-problem.
    for (int d = 0; d < 2; d += 1) {
      cv::Mat dst = z.col(d);
      findSmoothestPath(x.col(d), u.col(d), rho, lambda, dst);
    }
    //LOG(INFO) << "z-update: norm(x - z) => " << cv::norm(x - z) / n;

    // Update multipliers.
    u += x - z;

    cv::Mat r = x - z;
    cv::Mat s = rho * (z - z_old);

    LOG(INFO) << "norm(r) => " << cv::norm(r);
    LOG(INFO) << "norm(s) => " << cv::norm(s);
    LOG(INFO) << "norm(x - x_star) / n => " << cv::norm(x - x_star) / n;

    // Play the video.
    for (int t = 0; t < n; t += 1) {
      cv::Mat display = color_images[t].clone();

      // Draw tracks on frame.
      cv::Point2d pt1;
      cv::Point2d pt2;

      pt1 = cv::Point2d(x_star.at<double>(t, 0), x_star.at<double>(t, 1));
      pt2 = pt1 + cv::Point2d(DIAMETER, DIAMETER);
      cv::rectangle(display, pt1, pt2, cv::Scalar(0, 0x99, 0), 2);

      pt1 = cv::Point2d(z.at<double>(t, 0), z.at<double>(t, 1));
      pt2 = pt1 + cv::Point2d(DIAMETER, DIAMETER);
      cv::rectangle(display, pt1, pt2, cv::Scalar(0xFF, 0, 0), 2);

      pt1 = cv::Point2d(x.at<double>(t, 0), x.at<double>(t, 1));
      pt2 = pt1 + cv::Point2d(DIAMETER, DIAMETER);
      cv::rectangle(display, pt1, pt2, cv::Scalar(0, 0, 0xFF), 2);

      cv::imshow("video", display);
      cv::waitKey(10);
    }

    if (rho < 1.) {
      cv::Mat y = rho * u;
      rho = rho * FLAGS_rho_step;
      u = 1. / rho * y;
    }
  }

  return 0;
}
