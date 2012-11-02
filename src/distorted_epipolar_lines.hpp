#include <vector>
#include <opencv2/core/core.hpp>
#include "camera_properties.hpp"

class DistortedEpipolarRasterizer {
  public:
    DistortedEpipolarRasterizer(const CameraProperties& camera2,
                                const cv::Matx33d& F);

    void init();

    // Computes an epipolar line given 
    void compute(const cv::Point2d& x, std::vector<cv::Point>& line) const;

  private:
    const CameraProperties* camera_;
    cv::Mat F_;
    double radius_;
};

