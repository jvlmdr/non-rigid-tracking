#ifndef SPACE_TIME_IMAGE_POINT_HPP_
#define SPACE_TIME_IMAGE_POINT_HPP_

class SpaceTimeImagePoint {
  public:
    cv::Point2d p;
    int t;

    SpaceTimeImagePoint() : p(0, 0), t(0) {}
    SpaceTimeImagePoint(const cv::Point2d& p, int t) : p(p), t(t) {}

    inline double x() const { return p.x; }
    inline double y() const { return p.y; }
};

#endif
