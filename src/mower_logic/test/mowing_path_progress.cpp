#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "../src/mower_logic/behaviors/MowingPathProgress.h"

namespace {

std::vector<geometry_msgs::PoseStamped> makePath(const std::vector<std::pair<double, double>>& points) {
  std::vector<geometry_msgs::PoseStamped> path(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    path[i].pose.position.x = points[i].first;
    path[i].pose.position.y = points[i].second;
  }
  return path;
}

TEST(MowingPathProgress, DoesNotJumpAcrossNearbyHairpin) {
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i <= 20; ++i) points.emplace_back(i * 0.1, 0.0);
  points.emplace_back(2.05, 0.05);
  points.emplace_back(2.0, 0.1);
  for (int i = 19; i >= 0; --i) points.emplace_back(i * 0.1, 0.1);
  const auto path = makePath(points);
  EXPECT_EQ(12, orderedMowingPathIndex(path, 12, 1.0, 0.1));
  EXPECT_EQ(-1, orderedMowingPathIndex(path, 12, 2.0, 0.1));
}

TEST(MowingPathProgress, AdvancesInOrderThroughHairpin) {
  const auto path = makePath({{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0}, {0.2, 0.1},
                              {0.1, 0.1}, {0.0, 0.1}});
  int index = 0;
  for (size_t i = 0; i < path.size(); ++i) {
    const auto& point = path[i].pose.position;
    index = orderedMowingPathIndex(path, index, point.x, point.y);
    EXPECT_EQ(static_cast<int>(i), index);
  }
}

TEST(MowingPathProgress, AdvancesPastInsideOfObservedTightTurn) {
  const auto path = makePath({{-17.571, -5.931}, {-17.637, -6.003}, {-17.567, -5.968},
                              {-17.480, -5.919}, {-17.391, -5.872}, {-17.303, -5.826},
                              {-17.216, -5.777}});
  int index = 0;
  index = orderedMowingPathIndex(path, index, -17.447, -5.889);
  EXPECT_EQ(3, index);
  index = orderedMowingPathIndex(path, index, -17.314, -5.823);
  EXPECT_EQ(5, index);
  index = orderedMowingPathIndex(path, index, -17.174, -5.751);
  EXPECT_EQ(6, index);
}

TEST(MowingPathProgress, AdvancesPastInsideOfObservedSecondTurn) {
  const auto path = makePath({{-17.168, 3.900}, {-17.227, 3.820}, {-17.165, 3.791},
                              {-17.069, 3.762}, {-16.974, 3.729}});
  int index = orderedMowingPathIndex(path, 0, -16.985, 3.820);
  EXPECT_EQ(2, index);
  index = orderedMowingPathIndex(path, index, -16.920, 3.794);
  EXPECT_EQ(4, index);
  index = orderedMowingPathIndex(path, index, -16.856, 3.767);
  EXPECT_EQ(4, index);
}

TEST(MowingPathProgress, TracksRecordedTurnRecovery) {
  const auto path = makePath({{0.0, 0.0}, {-0.011423, 0.099345}, {-0.022879, 0.198686},
                              {-0.048826, 0.236995}, {-0.104108, 0.153665},
                              {-0.159953, 0.070715}, {-0.216501, -0.011761},
                              {-0.273188, -0.094142}, {-0.332398, -0.174717},
                              {-0.371345, -0.265808}, {-0.397285, -0.362385}});
  int index = 0;
  EXPECT_EQ(0, orderedMowingPathIndex(path, index, -0.256382, -0.008460));
  index = orderedMowingPathIndex(path, index, -0.278539, 0.019126);
  EXPECT_EQ(4, index);
  index = orderedMowingPathIndex(path, index, -0.288485, 0.020082);
  EXPECT_EQ(6, index);
  index = orderedMowingPathIndex(path, index, -0.283561, -0.029989);
  EXPECT_EQ(7, index);
  index = orderedMowingPathIndex(path, index, -0.325924, -0.126003);
  EXPECT_EQ(8, index);
  index = orderedMowingPathIndex(path, index, -0.359223, -0.225560);
  EXPECT_EQ(9, index);
  index = orderedMowingPathIndex(path, index, -0.388204, -0.326480);
  EXPECT_EQ(10, index);
}

TEST(MowingPathProgress, CornerLookaheadIsLimitedByPathLength) {
  const auto path = makePath({{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0}, {0.3, 0.0},
                              {0.3, 0.1}, {0.2, 0.1}, {0.1, 0.1}, {0.0, 0.1}});
  int index = 1;
  for (int i = 0; i < 10; ++i) {
    index = orderedMowingPathIndex(path, index, 0.1, 0.1);
    EXPECT_EQ(1, index);
  }
}

TEST(MowingPathProgress, AdvancesOnStraightPathWithoutSearchingAcrossDistanceIncrease) {
  const auto path = makePath({{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0}, {0.3, 0.0}, {0.4, 0.0}});
  EXPECT_EQ(2, orderedMowingPathIndex(path, 0, 0.21, 0.02));
  EXPECT_EQ(2, orderedMowingPathIndex(path, 2, 0.11, 0.0));
  EXPECT_EQ(-1, orderedMowingPathIndex(path, 0, 0.4, 0.0));
}

TEST(MowingPathProgress, EqualDistanceDoesNotAdvanceButDuplicatesDo) {
  const auto path = makePath({{0.0, 0.0}, {0.0, 0.0}, {0.2, 0.0}});
  EXPECT_EQ(1, orderedMowingPathIndex(path, 0, 0.1, 0.0));
  EXPECT_EQ(2, orderedMowingPathIndex(path, 1, 0.11, 0.0));
}

TEST(MowingPathProgress, RejectsInvalidInputs) {
  const auto path = makePath({{0.0, 0.0}, {0.1, 0.0}});
  EXPECT_EQ(-1, orderedMowingPathIndex(path, -1, 0.0, 0.0));
  EXPECT_EQ(-1, orderedMowingPathIndex(path, 2, 0.0, 0.0));
  EXPECT_EQ(-1, orderedMowingPathIndex(path, 0, std::numeric_limits<double>::quiet_NaN(), 0.0));
  auto invalid = path;
  invalid[1].pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(-1, orderedMowingPathIndex(invalid, 0, 0.0, 0.0));
}

TEST(MowingPathProgress, HandlesPathEnd) {
  const auto path = makePath({{0.0, 0.0}, {0.1, 0.0}});
  EXPECT_EQ(1, orderedMowingPathIndex(path, 1, 0.1, 0.0));
}

TEST(MowingPathProgress, CompletionRequiresSuccessAndProgressNearEnd) {
  EXPECT_FALSE(mowingPathCompleted(98, 100, false));
  EXPECT_FALSE(mowingPathCompleted(100, 100, false));
  EXPECT_FALSE(mowingPathCompleted(95, 100, true));
  EXPECT_FALSE(mowingPathCompleted(-1, 100, true));
  EXPECT_TRUE(mowingPathCompleted(96, 100, true));
  EXPECT_TRUE(mowingPathCompleted(100, 100, true));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
