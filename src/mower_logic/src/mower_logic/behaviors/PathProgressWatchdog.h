#ifndef MOWER_LOGIC_PATH_PROGRESS_WATCHDOG_H
#define MOWER_LOGIC_PATH_PROGRESS_WATCHDOG_H

class PathProgressWatchdog {
 public:
  PathProgressWatchdog(int index, double now) : last_index_(index), last_progress_time_(now) {
  }

  bool update(int index, double now, double timeout) {
    if (index > last_index_) {
      last_index_ = index;
      last_progress_time_ = now;
    }
    if (timeout <= 0.0 || now < last_progress_time_) {
      last_progress_time_ = now;
      return false;
    }
    return now - last_progress_time_ >= timeout;
  }

 private:
  int last_index_;
  double last_progress_time_;
};

#endif
