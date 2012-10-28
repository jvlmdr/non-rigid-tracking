#include <opencv2/core/core.hpp>

// Returns the sum of the two residuals.
// Hartley and Sturm, "Triangulation", CVIU 1997.
// Hartley and Zisserman, 2nd ed. p318.
//
// Convention used is x2^T F x1 = 0.
// "First" image co-ordinates multiplied on the right.
double optimalTriangulation(cv::Point2d& x1,
                            cv::Point2d& x2,
                            const cv::Matx33d& F);
