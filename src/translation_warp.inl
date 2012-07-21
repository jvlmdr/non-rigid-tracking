template<class T>
bool TranslationWarpFunction::operator()(const T* const x,
                                         const T* const p,
                                         T* q) const {
  // x' = x + tx
  // y' = y + ty
  q[0] = x[0] + p[0];
  q[1] = x[1] + p[1];

  return true;
}
