#include "feature_sets.hpp"
#include "gtest/gtest.h"

template<class Key, class Value>
bool operator==(const std::map<Key, Value>& lhs,
                const std::map<Key, Value>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }

  typename std::map<Key, Value>::const_iterator x = lhs.begin();
  typename std::map<Key, Value>::const_iterator y = rhs.begin();

  while (x != lhs.end() && y != rhs.end()) {
    if (x->first != y->first) {
      return false;
    }

    if (x->second != y->second) {
      return false;
    }

    ++x;
    ++y;
  }

  // Both must have ended simultaneously.
  return x == lhs.end() && y == rhs.end();
}

template<class Key, class Value>
bool operator!=(const std::map<Key, Value>& lhs,
                const std::map<Key, Value>& rhs) {
  return !(lhs == rhs);
}

TEST(FeatureSets, Count) {
  std::vector<ImageIndex> vertices(4);
  vertices[0] = ImageIndex(0, 0);
  vertices[1] = ImageIndex(0, 0);
  vertices[2] = ImageIndex(0, 1);
  vertices[3] = ImageIndex(0, 2);

  // Check that there are four sets.
  FeatureSets<int> sets;
  sets.init(vertices);

  ASSERT_EQ(sets.count(), 4);
}

TEST(FeatureSets, Join) {
  std::vector<ImageIndex> vertices(4);
  vertices[0] = ImageIndex(0, 0);
  vertices[1] = ImageIndex(0, 0);
  vertices[2] = ImageIndex(0, 1);
  vertices[3] = ImageIndex(0, 2);

  FeatureSets<int> sets;
  sets.init(vertices);
  sets.join(0, 2);

  ASSERT_EQ(sets.count(), 3);

  std::map<ImageIndex, int> desired;
  desired[ImageIndex(0, 0)] = 0;
  desired[ImageIndex(0, 1)] = 2;
  ASSERT_EQ(sets.find(0).elements, desired);
  ASSERT_EQ(sets.find(2).elements, desired);

  desired.clear();
  desired[ImageIndex(0, 0)] = 1;
  ASSERT_EQ(sets.find(1).elements, desired);

  desired.clear();
  desired[ImageIndex(0, 2)] = 3;
  ASSERT_EQ(sets.find(3).elements, desired);
}

TEST(FeatureSets, SeparateJoin) {
  std::vector<ImageIndex> vertices(4);
  vertices[0] = ImageIndex(0, 0);
  vertices[1] = ImageIndex(0, 0);
  vertices[2] = ImageIndex(0, 1);
  vertices[3] = ImageIndex(0, 2);

  FeatureSets<int> sets;
  sets.init(vertices);
  sets.join(0, 2);
  sets.join(1, 3);

  // There should be only two sets.
  ASSERT_EQ(sets.count(), 2);

  std::map<ImageIndex, int> desired;
  desired[ImageIndex(0, 0)] = 0;
  desired[ImageIndex(0, 1)] = 2;
  ASSERT_EQ(sets.find(0).elements, desired);
  ASSERT_EQ(sets.find(2).elements, desired);

  desired.clear();
  desired[ImageIndex(0, 0)] = 1;
  desired[ImageIndex(0, 2)] = 3;
  ASSERT_EQ(sets.find(1).elements, desired);
  ASSERT_EQ(sets.find(3).elements, desired);
}

TEST(FeatureSets, CompoundJoin) {
  std::vector<ImageIndex> vertices(4);
  vertices[0] = ImageIndex(0, 0);
  vertices[1] = ImageIndex(0, 0);
  vertices[2] = ImageIndex(0, 1);
  vertices[3] = ImageIndex(0, 2);

  FeatureSets<int> sets;
  sets.init(vertices);
  sets.join(0, 2);
  sets.join(2, 3);

  // There should be only two sets.
  ASSERT_EQ(sets.count(), 2);

  std::map<ImageIndex, int> desired;
  desired[ImageIndex(0, 0)] = 0;
  desired[ImageIndex(0, 1)] = 2;
  desired[ImageIndex(0, 2)] = 3;
  ASSERT_EQ(sets.find(0).elements, desired);
  ASSERT_EQ(sets.find(2).elements, desired);
  ASSERT_EQ(sets.find(3).elements, desired);

  desired.clear();
  desired[ImageIndex(0, 0)] = 1;
  ASSERT_EQ(sets.find(1).elements, desired);
}

TEST(FeatureSets, RedundantJoin) {
  std::vector<ImageIndex> vertices(4);
  vertices[0] = ImageIndex(0, 0);
  vertices[1] = ImageIndex(0, 0);
  vertices[2] = ImageIndex(0, 1);
  vertices[3] = ImageIndex(0, 2);

  FeatureSets<int> sets;
  sets.init(vertices);
  sets.join(0, 2);
  sets.join(2, 3);
  sets.join(0, 3);

  // There should be only two sets.
  ASSERT_EQ(sets.count(), 2);

  std::map<ImageIndex, int> desired;
  desired[ImageIndex(0, 0)] = 0;
  desired[ImageIndex(0, 1)] = 2;
  desired[ImageIndex(0, 2)] = 3;
  ASSERT_EQ(sets.find(0).elements, desired);
  ASSERT_EQ(sets.find(2).elements, desired);
  ASSERT_EQ(sets.find(3).elements, desired);

  desired.clear();
  desired[ImageIndex(0, 0)] = 1;
  ASSERT_EQ(sets.find(1).elements, desired);
}

TEST(FeatureSets, Compatible) {
  std::vector<ImageIndex> vertices(4);
  vertices[0] = ImageIndex(0, 0);
  vertices[1] = ImageIndex(0, 0);
  vertices[2] = ImageIndex(0, 1);
  vertices[3] = ImageIndex(0, 2);

  FeatureSets<int> sets;
  sets.init(vertices);

  ASSERT_EQ(sets.compatible(0, 1), false);
  ASSERT_EQ(sets.compatible(0, 2), true);
  ASSERT_EQ(sets.compatible(0, 3), true);
  ASSERT_EQ(sets.compatible(1, 2), true);
  ASSERT_EQ(sets.compatible(1, 3), true);
  ASSERT_EQ(sets.compatible(2, 3), true);

  sets.join(0, 2);

  ASSERT_EQ(sets.compatible(0, 1), false);
  ASSERT_EQ(sets.compatible(0, 3), true);
  ASSERT_EQ(sets.compatible(1, 2), false);
  ASSERT_EQ(sets.compatible(1, 3), true);
  ASSERT_EQ(sets.compatible(2, 3), true);

  sets.join(1, 3);

  ASSERT_EQ(sets.compatible(0, 1), false);
  ASSERT_EQ(sets.compatible(0, 3), false);
  ASSERT_EQ(sets.compatible(1, 2), false);
  ASSERT_EQ(sets.compatible(2, 3), false);
}
