#!/usr/bin/env python
import math
import threading

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Ld19SafetyStop:
    def __init__(self):
        self.stop_distance = rospy.get_param("~stop_distance", 0.2) # was 0.65
        self.clear_distance = rospy.get_param("~clear_distance", 0.4) # was 0.85
        self.clear_time = rospy.get_param("~clear_time", 2.0)
        self.scan_timeout = rospy.get_param("~scan_timeout", 0.5)
        self.publish_rate = rospy.get_param("~publish_rate", 20.0)

        values = (
            self.stop_distance,
            self.clear_distance,
            self.clear_time,
            self.scan_timeout,
            self.publish_rate,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("LD19 safety parameters must be finite")
        if (
            self.stop_distance <= 0.0
            or self.clear_distance <= self.stop_distance
            or self.clear_time < 0.0
            or self.scan_timeout <= 0.0
            or self.publish_rate <= 0.0
        ):
            raise ValueError("Invalid LD19 safety distance, timeout, or rate parameters")

        self.lock = threading.Lock()
        self.active = True
        self.last_scan_time = None
        self.clear_since = None
        self.last_reported_state = None

        self.lock_publisher = rospy.Publisher("/ld19_safety/lock", Bool, queue_size=1)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.on_timer)

    def on_scan(self, scan):
        if (
            not scan.ranges
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or scan.range_min < 0.0
            or scan.range_max <= scan.range_min
        ):
            rospy.logwarn_throttle(5.0, "LD19 safety: ignoring scan with invalid range limits or no readings")
            return

        nearest = None
        has_valid_range = False
        for distance in scan.ranges:
            if math.isnan(distance) or (math.isinf(distance) and distance < 0.0):
                continue
            if math.isinf(distance):
                has_valid_range = True
                continue
            if distance < scan.range_min:
                has_valid_range = True
                nearest = 0.0
                break
            if distance > scan.range_max:
                continue
            has_valid_range = True
            if nearest is None or distance < nearest:
                nearest = distance

        if not has_valid_range:
            rospy.logwarn_throttle(5.0, "LD19 safety: ignoring scan with no usable readings")
            return

        now = rospy.Time.now()
        with self.lock:
            if self.last_scan_time is None or now - self.last_scan_time > rospy.Duration(self.scan_timeout):
                self.active = True
                self.clear_since = None
            self.last_scan_time = now
            if nearest is not None and nearest <= self.stop_distance:
                self.active = True
                self.clear_since = None
            elif self.active:
                if nearest is not None and nearest <= self.clear_distance:
                    self.clear_since = None
                elif self.clear_since is None:
                    self.clear_since = now

    def on_timer(self, _event):
        now = rospy.Time.now()
        with self.lock:
            scan_stale = self.last_scan_time is None or now - self.last_scan_time > rospy.Duration(
                self.scan_timeout
            )
            if scan_stale:
                self.active = True
                self.clear_since = None
            elif (
                self.active
                and self.clear_since is not None
                and now - self.clear_since >= rospy.Duration(self.clear_time)
            ):
                self.active = False
                self.clear_since = None

            active = self.active

        self.lock_publisher.publish(Bool(data=active))
        if active != self.last_reported_state:
            if active:
                rospy.logwarn("LD19 safety stop active: obstacle detected or scan is stale")
            else:
                rospy.loginfo("LD19 safety stop cleared after the scan area remained clear")
            self.last_reported_state = active


if __name__ == "__main__":
    rospy.init_node("ld19_safety_stop")
    Ld19SafetyStop()
    rospy.spin()
