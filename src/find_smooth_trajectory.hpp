#ifndef FIND_SMOOTH_TRAJECTORY_HPP_
#define FIND_SMOOTH_TRAJECTORY_HPP_

#include <opencv2/core/core.hpp>
#include "multiview_track.hpp"
#include "track.hpp"

// A single observation of one point.
struct PointObservation {
  cv::Matx34d P;
  cv::Point2d w;
};

// Finds the smoothest trajectory which satisfies the given projections.
//
// The 3D trajectory will be computed for all frames [a, b], where the first
// projection is in frame a and the last projection is in frame b. This
// trajectory will have length (b - a + 1).
//
// The function also outputs the final residual (after non-linear refinement)
// and the condition of the initial linear system.
//
// Cameras are supplied as 3x4 matrices (with intrinsics applied).
// If cameras are not identical, they may need to be normalized.
// Assumes all cameras are known (unnecessarily strong?)
void findSmoothTrajectory(const MultiviewTrack<PointObservation>& observations,
                          double lambda,
                          Track<cv::Point3d>& trajectory,
                          double& residual,
                          double& condition);

#endif
