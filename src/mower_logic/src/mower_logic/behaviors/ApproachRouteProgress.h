#ifndef MOWER_LOGIC_APPROACH_ROUTE_PROGRESS_H
#define MOWER_LOGIC_APPROACH_ROUTE_PROGRESS_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

class ApproachRouteProgress {
 public:
  bool setPath(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& target, double x, double y) {
    if (path.header.frame_id != target.header.frame_id || path.header.frame_id.empty() || path.poses.size() < 2 ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(target.pose.position.x) ||
        !std::isfinite(target.pose.position.y))
      return false;
    const auto& goal = path.poses.back().pose.position;
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y) ||
        std::hypot(goal.x - target.pose.position.x, goal.y - target.pose.position.y) > 0.5)
      return false;

    std::vector<double> lengths(path.poses.size(), 0.0);
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto& point = path.poses[i].pose.position;
      if ((!path.poses[i].header.frame_id.empty() && path.poses[i].header.frame_id != path.header.frame_id) ||
          !std::isfinite(point.x) || !std::isfinite(point.y))
        return false;
      if (i > 0) {
        const auto& previous = path.poses[i - 1].pose.position;
        lengths[i] = lengths[i - 1] + std::hypot(point.x - previous.x, point.y - previous.y);
        if (!std::isfinite(lengths[i])) return false;
      }
    }
    if (lengths.back() <= 0.0) return false;

    size_t segment = 0;
    double progress = 0.0;
    if (!project(path, lengths, x, y, segment, progress)) return false;
    size_t old_segment = segment_;
    double old_progress = 0.0;
    // Keep the existing route while it is still nearby, so repeated replans
    // cannot count the same back-and-forth movement as new progress.
    if (!path_.poses.empty() && project(path_, lengths_, x, y, old_segment, old_progress)) {
      pending_path_ = path;
      pending_lengths_ = std::move(lengths);
      tracking_ = true;
      return true;
    }
    path_ = path;
    lengths_ = std::move(lengths);
    segment_ = segment;
    best_progress_ = progress;
    pending_path_.poses.clear();
    pending_lengths_.clear();
    pending_progress_ = 0.0;
    anchor_x_ = x;
    anchor_y_ = y;
    tracking_ = true;
    return true;
  }

  void clearPath() {
    path_.poses.clear();
    lengths_.clear();
    pending_path_.poses.clear();
    pending_lengths_.clear();
    pending_progress_ = 0.0;
    tracking_ = false;
  }

  bool tracking() const {
    return tracking_;
  }

  bool update(double x, double y) {
    if (path_.poses.empty() || !std::isfinite(x) || !std::isfinite(y)) {
      tracking_ = false;
      return false;
    }
    size_t segment = segment_;
    double progress = 0.0;
    if (!project(path_, lengths_, x, y, segment, progress)) {
      segment = 0;
      if (pending_path_.poses.empty() || !project(pending_path_, pending_lengths_, x, y, segment, progress)) {
        tracking_ = false;
        return false;
      }
      path_ = std::move(pending_path_);
      lengths_ = std::move(pending_lengths_);
      pending_path_.poses.clear();
      pending_lengths_.clear();
      segment_ = segment;
      best_progress_ = progress;
      pending_progress_ = 0.0;
      anchor_x_ = x;
      anchor_y_ = y;
      tracking_ = true;
      return false;
    }
    tracking_ = true;
    if (progress > best_progress_) {
      pending_progress_ += progress - best_progress_;
      best_progress_ = progress;
      segment_ = segment;
    }
    if (pending_progress_ >= 0.1 && std::hypot(x - anchor_x_, y - anchor_y_) >= 0.1) {
      pending_progress_ = 0.0;
      anchor_x_ = x;
      anchor_y_ = y;
      return true;
    }
    return false;
  }

 private:
  static bool project(const nav_msgs::Path& path, const std::vector<double>& lengths, double x, double y,
                      size_t& segment, double& progress) {
    double best_squared_distance = std::numeric_limits<double>::infinity();
    size_t best_segment = segment;
    double best_progress = 0.0;
    for (size_t i = segment; i + 1 < path.poses.size(); ++i) {
      const auto& a = path.poses[i].pose.position;
      const auto& b = path.poses[i + 1].pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double squared_length = dx * dx + dy * dy;
      if (!std::isfinite(squared_length)) return false;
      if (squared_length <= 1e-12) continue;
      const double t = std::max(0.0, std::min(1.0, ((x - a.x) * dx + (y - a.y) * dy) / squared_length));
      const double offset_x = x - (a.x + t * dx);
      const double offset_y = y - (a.y + t * dy);
      const double squared_distance = offset_x * offset_x + offset_y * offset_y;
      if (squared_distance > best_squared_distance) break;
      if (squared_distance < best_squared_distance) {
        best_squared_distance = squared_distance;
        best_segment = i;
        best_progress = lengths[i] + t * std::sqrt(squared_length);
      }
    }
    if (best_squared_distance > 0.5 * 0.5) return false;
    segment = best_segment;
    progress = best_progress;
    return true;
  }

  nav_msgs::Path path_;
  std::vector<double> lengths_;
  nav_msgs::Path pending_path_;
  std::vector<double> pending_lengths_;
  size_t segment_ = 0;
  double best_progress_ = 0.0;
  double pending_progress_ = 0.0;
  double anchor_x_ = 0.0;
  double anchor_y_ = 0.0;
  bool tracking_ = false;
};

#endif
