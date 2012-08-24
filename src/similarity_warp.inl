#include <cmath>

template<class T>
bool SimilarityWarpFunction::operator()(const T* const x,
                                        const T* const p,
                                        T* q) const {
  // x' = scale * (cos(theta) * x + -sin(theta) * y) + tx
  // y' = scale * (sin(theta) * x +  cos(theta) * y) + ty
  q[0] = exp(p[2]) * (cos(p[3]) * x[0] - sin(p[3]) * x[1]) + p[0];
  q[1] = exp(p[2]) * (sin(p[3]) * x[0] + cos(p[3]) * x[1]) + p[1];

  return true;
}