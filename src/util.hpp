#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

inline double sqr(double x);
inline bool isNumber(double x);
inline bool isFinite(double x);

// Differentiate between 2D and 3D points as "image" and "world" points.
inline cv::Mat imagePointToHomogeneous(const cv::Point2d& x);
inline cv::Point2d imagePointFromHomogeneous(const cv::Mat& X);
inline cv::Mat worldPointToHomogeneous(const cv::Point3d& x);
inline cv::Point3d worldPointFromHomogeneous(const cv::Mat& X);

inline void imagePointsToHomogeneous(const std::vector<cv::Point2d>& points,
                                     cv::Mat& matrix);
inline void imagePointsFromHomogeneous(const cv::Mat& matrix,
                                       std::vector<cv::Point2d>& points);

inline cv::Point2d affineTransformImagePoint(const cv::Point2d& x,
                                             const cv::Mat& A);

double cond(const cv::Mat& A);

template<class T>
T* takeAddress(T& x);

template<class U, class T>
U cast(const T& t);

template<class U, class T>
U* takeAddressAndCast(T& x);

bool fileExists(const std::string& filename);

#include "util.inl"

#endif
