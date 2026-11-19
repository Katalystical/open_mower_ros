#ifndef MOWER_LOGIC_APPROACH_PROGRESS_WATCHDOG_H
#define MOWER_LOGIC_APPROACH_PROGRESS_WATCHDOG_H

#include <cmath>
#include <limits>

class ApproachProgressWatchdog {
 public:
  explicit ApproachProgressWatchdog(double now) : last_progress_time_(now) {
  }

  bool update(double distance, double angle, double now, double timeout, bool route_progress = false) {
    if (timeout <= 0.0 || now < last_progress_time_) {
      last_progress_time_ = now;
      best_distance_ = std::numeric_limits<double>::infinity();
      best_angle_ = std::numeric_limits<double>::infinity();
    }

    if (route_progress) last_progress_time_ = now;
    if (std::isfinite(distance) && distance >= 0.0 && distance < best_distance_ - 0.1) {
      best_distance_ = distance;
      last_progress_time_ = now;
    }
    if (distance <= 0.3 && std::isfinite(angle) && angle >= 0.0 && angle < best_angle_ - 0.2) {
      best_angle_ = angle;
      last_progress_time_ = now;
    }
    return timeout > 0.0 && now - last_progress_time_ >= timeout;
  }

 private:
  double last_progress_time_;
  double best_distance_ = std::numeric_limits<double>::infinity();
  double best_angle_ = std::numeric_limits<double>::infinity();
};

#endif
