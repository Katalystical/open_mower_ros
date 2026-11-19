#ifndef MOWER_LOGIC_MOWING_PATH_PROGRESS_H
#define MOWER_LOGIC_MOWING_PATH_PROGRESS_H

#include <geometry_msgs/PoseStamped.h>

#include <algorithm>
#include <cmath>
#include <vector>

inline int orderedMowingPathIndex(const std::vector<geometry_msgs::PoseStamped>& path, int start_index, double x,
                                  double y) {
  if (start_index < 0 || static_cast<size_t>(start_index) >= path.size() || !std::isfinite(x) || !std::isfinite(y))
    return -1;

  const size_t start = static_cast<size_t>(start_index);
  const size_t end = std::min(path.size(), start + 25);
  constexpr double max_distance_squared = 0.3 * 0.3;
  auto distanceSquared = [x, y](const geometry_msgs::PoseStamped& pose) {
    const double dx = x - pose.pose.position.x;
    const double dy = y - pose.pose.position.y;
    return dx * dx + dy * dy;
  };

  double previous_distance = distanceSquared(path[start]);
  if (!std::isfinite(previous_distance) || previous_distance > max_distance_squared) return -1;

  size_t closest = start;
  for (size_t i = start + 1; i < end; ++i) {
    const double distance = distanceSquared(path[i]);
    if (!std::isfinite(distance)) return -1;
    if (distance > max_distance_squared || distance > previous_distance) break;
    if (distance < previous_distance) {
      closest = i;
    } else {
      const auto& previous = path[i - 1].pose.position;
      const auto& current = path[i].pose.position;
      if (current.x != previous.x || current.y != previous.y) break;
      closest = i;
    }
    previous_distance = distance;
  }

  // A mower can cut inside a corner, making an intermediate waypoint farther
  // away even though the next part of the same path is closer.
  constexpr double max_corner_lookahead = 0.3;
  const double closest_distance = distanceSquared(path[closest]);
  double path_distance = 0.0;
  for (size_t i = closest + 1; i < end; ++i) {
    const auto& previous = path[i - 1].pose.position;
    const auto& current = path[i].pose.position;
    const double segment_length = std::hypot(current.x - previous.x, current.y - previous.y);
    const double distance = distanceSquared(path[i]);
    if (!std::isfinite(segment_length) || !std::isfinite(distance)) return -1;
    path_distance += segment_length;
    if (path_distance > max_corner_lookahead) break;
    if (distance < closest_distance) return static_cast<int>(i);
  }
  return static_cast<int>(closest);
}

inline bool mowingPathCompleted(int index, size_t path_size, bool succeeded) {
  if (!succeeded || index < 0) return false;
  const size_t progress = static_cast<size_t>(index);
  return progress >= path_size || path_size - progress < 5;
}

#endif
