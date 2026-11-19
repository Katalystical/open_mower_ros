#ifndef MOWER_SIMULATION_VELOCITY_COMMAND_H
#define MOWER_SIMULATION_VELOCITY_COMMAND_H

#include <chrono>
#include <cmath>

class VelocityCommand {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  static constexpr double TIMEOUT_SECONDS = 1.0;

  bool set(double linear, double angular, TimePoint now) {
    if (!std::isfinite(linear) || !std::isfinite(angular)) {
      clear();
      return false;
    }
    linear_ = linear;
    angular_ = angular;
    last_command_ = now;
    active_ = true;
    return true;
  }

  bool expire(TimePoint now) {
    if (active_ &&
        (now < last_command_ || std::chrono::duration<double>(now - last_command_).count() >= TIMEOUT_SECONDS)) {
      const bool moving = linear_ != 0.0 || angular_ != 0.0;
      clear();
      return moving;
    }
    return false;
  }

  void clear() {
    linear_ = 0.0;
    angular_ = 0.0;
    active_ = false;
  }

  double linear() const {
    return linear_;
  }
  double angular() const {
    return angular_;
  }

 private:
  double linear_ = 0.0;
  double angular_ = 0.0;
  bool active_ = false;
  TimePoint last_command_{};
};

#endif
