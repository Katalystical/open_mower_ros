#include <gtest/gtest.h>
#include <mbf_msgs/ExePathAction.h>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "../src/mower_logic/StateSubscriber.h"
#include "../src/mower_logic/behaviors/ApproachProgressWatchdog.h"
#include "../src/mower_logic/behaviors/ApproachRouteProgress.h"
#include "../src/mower_logic/behaviors/PathProgressWatchdog.h"

namespace {

nav_msgs::Path makeRoute(const std::vector<std::pair<double, double>>& points) {
  nav_msgs::Path path;
  path.header.frame_id = "map";
  for (const auto& point : points) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    path.poses.push_back(pose);
  }
  return path;
}

TEST(StateSubscriber, ReturnsEachNewRouteMessageOnce) {
  StateSubscriber<mbf_msgs::ExePathActionGoal> subscriber("/unused");
  std::uint64_t sequence = 0;
  mbf_msgs::ExePathActionGoal message;
  EXPECT_FALSE(subscriber.getMessageSince(sequence, message));
  message.goal_id.id = "first";
  subscriber.setMessage(message);
  message.goal_id.id.clear();
  ASSERT_TRUE(subscriber.getMessageSince(sequence, message));
  EXPECT_EQ("first", message.goal_id.id);
  EXPECT_FALSE(subscriber.getMessageSince(sequence, message));
  message.goal_id.id = "second";
  subscriber.setMessage(message);
  ASSERT_TRUE(subscriber.getMessageSince(sequence, message));
  EXPECT_EQ("second", message.goal_id.id);
}

}  // namespace

TEST(ApproachProgressWatchdog, TimesOutDuringPendingOrInvalidPose) {
  const double invalid = std::numeric_limits<double>::infinity();
  ApproachProgressWatchdog watchdog(100.0);
  EXPECT_FALSE(watchdog.update(invalid, invalid, 159.9, 60.0));
  EXPECT_TRUE(watchdog.update(invalid, invalid, 160.0, 60.0));
}

TEST(ApproachProgressWatchdog, RequiresMeaningfulImprovement) {
  ApproachProgressWatchdog watchdog(0.0);
  EXPECT_FALSE(watchdog.update(2.0, 1.0, 0.0, 60.0));
  EXPECT_FALSE(watchdog.update(1.95, 0.2, 30.0, 60.0));
  EXPECT_FALSE(watchdog.update(1.89, 0.1, 50.0, 60.0));
  EXPECT_FALSE(watchdog.update(2.1, 0.0, 109.9, 60.0));
  EXPECT_TRUE(watchdog.update(1.9, 0.0, 110.0, 60.0));
}

TEST(ApproachProgressWatchdog, CountsRotationOnlyNearGoal) {
  ApproachProgressWatchdog watchdog(0.0);
  EXPECT_FALSE(watchdog.update(0.29, 1.0, 0.0, 60.0));
  EXPECT_FALSE(watchdog.update(0.29, 0.9, 30.0, 60.0));
  EXPECT_FALSE(watchdog.update(0.29, 0.7, 50.0, 60.0));
  EXPECT_TRUE(watchdog.update(0.29, 0.6, 110.0, 60.0));
}

TEST(ApproachProgressWatchdog, DisabledAndClockRewindRestartTimeout) {
  ApproachProgressWatchdog watchdog(100.0);
  EXPECT_FALSE(watchdog.update(1.0, 0.0, 200.0, 0.0));
  EXPECT_FALSE(watchdog.update(1.0, 0.0, 1.0, 60.0));
  EXPECT_FALSE(watchdog.update(1.0, 0.0, 60.9, 60.0));
  EXPECT_TRUE(watchdog.update(1.0, 0.0, 61.0, 60.0));
}

TEST(ApproachProgressWatchdog, TracksRouteProgressDuringDetour) {
  const auto path = makeRoute({{0, 0}, {1, 0}, {1, 4}, {5, 4}, {5, 0}});
  const auto target = path.poses.back();
  ApproachRouteProgress route;
  ASSERT_TRUE(route.setPath(path, target, 0, 0));
  ApproachProgressWatchdog watchdog(0.0);
  const std::vector<std::pair<double, double>> positions = {{0, 0}, {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4},
                                                            {2, 4}, {3, 4}, {4, 4}, {5, 4}, {5, 3}};
  for (size_t i = 0; i < positions.size(); ++i) {
    const auto& position = positions[i];
    const double distance = std::hypot(position.first - 5, position.second);
    EXPECT_FALSE(watchdog.update(distance, 1.0, 20.0 * i, 60.0, route.update(position.first, position.second)))
        << "position " << i;
  }
  EXPECT_TRUE(watchdog.update(std::hypot(0.0, 3.0), 1.0, 260.0, 60.0, route.update(5, 3)));
}

TEST(ApproachProgressWatchdog, DoesNotPauseOnRecordedDetour) {
  // The recorded track is translated to the goal; the planned route was not retained.
  const std::vector<std::pair<double, double>> track = {
      {12.232, 5.457}, {11.967, 5.669},  {10.469, 6.395},  {9.922, 6.792},  {9.296, 7.494},  {8.106, 9.604},
      {7.162, 11.395}, {6.199, 13.179},  {5.094, 14.861},  {4.531, 15.556}, {4.122, 15.728}, {3.346, 15.566},
      {1.049, 14.398}, {-0.699, 13.522}, {-1.352, 13.418}, {-1.831, 13.092}};
  auto route_points = track;
  route_points.emplace_back(0.0, 0.0);
  const auto path = makeRoute(route_points);
  ApproachRouteProgress route;
  ASSERT_TRUE(route.setPath(path, path.poses.back(), track[0].first, track[0].second));
  ApproachProgressWatchdog watchdog(0.0);
  ApproachProgressWatchdog distance_only(0.0);
  bool distance_only_paused = false;
  double time = 0.0;
  for (size_t i = 1; i < track.size(); ++i) {
    const auto& a = track[i - 1];
    const auto& b = track[i];
    const double length = std::hypot(b.first - a.first, b.second - a.second);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
    for (int step = 1; step <= steps; ++step) {
      const double fraction = static_cast<double>(step) / steps;
      const double x = a.first + fraction * (b.first - a.first);
      const double y = a.second + fraction * (b.second - a.second);
      time += length / steps / 0.25;
      const double distance = std::hypot(x, y);
      distance_only_paused |= distance_only.update(distance, 1.0, time, 60.0);
      ASSERT_FALSE(watchdog.update(distance, 1.0, time, 60.0, route.update(x, y))) << "time " << time;
    }
  }
  EXPECT_TRUE(distance_only_paused);
}

TEST(ApproachRouteProgress, ReplanningDoesNotTurnOscillationIntoProgress) {
  const auto path = makeRoute({{0, 0}, {5, 0}});
  const auto target = path.poses.back();
  ApproachRouteProgress route;
  ASSERT_TRUE(route.setPath(path, target, 0, 0));
  ApproachProgressWatchdog watchdog(0.0);
  EXPECT_FALSE(watchdog.update(5.0, 1.0, 0.0, 60.0, route.update(0, 0)));
  ASSERT_TRUE(route.setPath(path, target, 0.2, 0));
  EXPECT_FALSE(watchdog.update(5.0, 1.0, 1.0, 60.0, route.update(0.2, 0)));
  for (int time = 2; time < 61; ++time) {
    const double x = time % 2 == 0 ? 0.0 : 0.2;
    ASSERT_TRUE(route.setPath(path, target, x, 0));
    EXPECT_FALSE(watchdog.update(5.0, 1.0, time, 60.0, route.update(x, 0))) << time;
  }
  EXPECT_TRUE(watchdog.update(5.0, 1.0, 61.0, 60.0, route.update(0.2, 0)));
}

TEST(ApproachRouteProgress, FollowsReplannedRouteAfterLeavingOriginal) {
  const auto original = makeRoute({{0, 0}, {5, 0}});
  const auto detour = makeRoute({{0, 0}, {0, 1}, {5, 1}, {5, 0}});
  ApproachRouteProgress route;
  ASSERT_TRUE(route.setPath(original, original.poses.back(), 0, 0));
  ASSERT_TRUE(route.setPath(detour, original.poses.back(), 0, 0));
  EXPECT_FALSE(route.update(0, 0.6));
  EXPECT_TRUE(route.update(0, 0.8));
}

TEST(ApproachRouteProgress, RejectsInvalidOrUnrelatedPlans) {
  const auto path = makeRoute({{0, 0}, {5, 0}});
  auto target = path.poses.back();
  ApproachRouteProgress route;
  EXPECT_FALSE(route.setPath(nav_msgs::Path(), target, 0, 0));
  target.pose.position.x = 6.0;
  EXPECT_FALSE(route.setPath(path, target, 0, 0));
  target = path.poses.back();
  target.header.frame_id = "other";
  EXPECT_FALSE(route.setPath(path, target, 0, 0));
  target = path.poses.back();
  EXPECT_FALSE(route.setPath(path, target, 0, 1.0));
  target.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(route.setPath(path, target, 0, 0));
  auto invalid_path = path;
  invalid_path.poses.front().header.frame_id = "other";
  EXPECT_FALSE(route.setPath(invalid_path, path.poses.back(), 0, 0));
  invalid_path = path;
  invalid_path.poses.front().pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(route.setPath(invalid_path, path.poses.back(), 0, 0));
  EXPECT_FALSE(route.tracking());
  EXPECT_FALSE(route.update(0.2, 0));
}

TEST(PathProgressWatchdog, TimesOutWithoutProgress) {
  PathProgressWatchdog watchdog(51, 100.0);
  EXPECT_FALSE(watchdog.update(51, 129.9, 30.0));
  EXPECT_TRUE(watchdog.update(51, 130.0, 30.0));
}

TEST(PathProgressWatchdog, ProgressRestartsTimeout) {
  PathProgressWatchdog watchdog(51, 100.0);
  EXPECT_FALSE(watchdog.update(52, 129.0, 30.0));
  EXPECT_FALSE(watchdog.update(51, 158.9, 30.0));
  EXPECT_TRUE(watchdog.update(52, 159.0, 30.0));
}

TEST(PathProgressWatchdog, DisabledWatchdogDoesNotAccumulateTime) {
  PathProgressWatchdog watchdog(0, 0.0);
  EXPECT_FALSE(watchdog.update(0, 100.0, 0.0));
  EXPECT_FALSE(watchdog.update(0, 101.0, 30.0));
  EXPECT_TRUE(watchdog.update(0, 130.0, 30.0));
}

TEST(PathProgressWatchdog, ClockRewindRestartsTimeout) {
  PathProgressWatchdog watchdog(0, 100.0);
  EXPECT_FALSE(watchdog.update(0, 1.0, 30.0));
  EXPECT_FALSE(watchdog.update(0, 30.9, 30.0));
  EXPECT_TRUE(watchdog.update(0, 31.0, 30.0));
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
