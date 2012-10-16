
UniqueDirectedMatch() : index(-1), distance(0), next_best(0) {}

UniqueDirectedMatch(int index, double distance, double next_best)
    : index(index), distance(distance), next_best(next_best) {}

UniqueDirectedMatch convertMatchPair(const RawMatchList& pair) {
  CHECK(pair.size() >= 2);
  return UniqueDirectedMatch(pair[0].index, pair[0].distance, pair[1].distance);
}

