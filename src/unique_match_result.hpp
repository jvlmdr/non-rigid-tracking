#ifndef UNIQUE_MATCH_RESULT_HPP_
#define UNIQUE_MATCH_RESULT_HPP_

// A MatchResult (compared to a simple Match) includes the distance.
// A UniqueMatchResult includes both second-best distances.
struct UniqueMatchResult {
  // Index of feature in image 1.
  int index1;
  // Index of feature in image 2.
  int index2;
  // Distance between feature appearances.
  double distance;
  // At least one of the following two variables will be true.
  // Matched from image 1 to image 2?
  bool forward;
  // Matched from image 2 to image 1?
  bool reverse;
  // Second-best distance matching from image 1 to image 2.
  double next_best_forward;
  // Second-best distance matching from image 2 to image 1.
  double next_best_reverse;

  UniqueMatchResult();
  UniqueMatchResult(int index1,
                    int index2,
                    double distance,
                    double next_best_forward,
                    double next_best_reverse);
  UniqueMatchResult(int index1,
                    int index2,
                    double distance,
                    bool forward,
                    double next_best);
  UniqueMatchResult(int index1,
                    int index2,
                    double distance);

  double minNextBest() const;

  UniqueMatchResult flip() const;
};

#endif
