#include <set>
#include <opencv2/core/core.hpp>
#include "camera_properties.hpp"

class DistortedEpipolarLineFinder {
  public:
    struct ComparePoints {
      bool operator()(const cv::Point& p, const cv::Point& q);
    };

    typedef std::set<cv::Point, ComparePoints> PixelSet;

    DistortedEpipolarLineFinder(const CameraProperties& camera2,
                                const cv::Matx33d& F);

    void init();

    // Computes an epipolar line given 
    void compute(const cv::Point2d& x, PixelSet& line) const;

  private:
    const CameraProperties* camera_;
    cv::Mat F_;
    double radius_;
};

