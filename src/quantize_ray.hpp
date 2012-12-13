#include <opencv2/core/core.hpp>
#include <vector>
#include "camera.hpp"

void quantizeRay(const cv::Point2d& projection,
                 const std::vector<Camera>& cameras,
                 int selected,
                 double delta,
                 std::map<double, cv::Point3d>& points);
