#ifndef RIGID_FEATURE_HPP_
#define RIGID_FEATURE_HPP_

#include "write.hpp"
#include "read.hpp"

struct RigidFeature {
  double x;
  double y;
  double size;
  double theta;

  RigidFeature();
  RigidFeature(double x, double y, double size, double theta);

  const double* data() const;
  double* data();
};

class WriteRigidFeature : public Write<RigidFeature> {
  public:
    ~WriteRigidFeature();
    void operator()(cv::FileStorage& file, const RigidFeature& x);
};

class ReadRigidFeature : public Read<RigidFeature> {
  public:
    ~ReadRigidFeature();
    void operator()(const cv::FileNode& node, RigidFeature& feature);
};

#endif
