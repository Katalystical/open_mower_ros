#!/usr/bin/env python3
import importlib.util
import os
import unittest
from unittest.mock import patch

import rospy
from sensor_msgs.msg import LaserScan


script = os.path.join(os.path.dirname(__file__), "../scripts/ld19_safety_stop.py")
spec = importlib.util.spec_from_file_location("ld19_safety_stop", script)
stop_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stop_module)


class SafetyStopTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        params = {"~stop_distance": 0.65, "~clear_distance": 0.85}
        patches = [
            patch.object(rospy.Time, "now", lambda: rospy.Time.from_sec(self.now)),
            patch.object(rospy, "Publisher"),
            patch.object(rospy, "Subscriber"),
            patch.object(rospy, "Timer"),
            patch.object(rospy, "get_param", lambda name, default: params.get(name, default)),
            patch.object(rospy, "logwarn_throttle"),
            patch.object(rospy, "logwarn"),
            patch.object(rospy, "loginfo"),
        ]
        for item in patches:
            item.start()
            self.addCleanup(item.stop)
        self.stop = stop_module.Ld19SafetyStop()

    def scan(self, ranges, minimum=0.02, maximum=12.0):
        return LaserScan(range_min=minimum, range_max=maximum, ranges=ranges)

    def clear_stop(self, reading=1.0):
        for _ in range(6):
            self.stop.on_scan(self.scan([reading]))
            self.stop.on_timer(None)
            self.now += 0.4
        self.assertFalse(self.stop.active)

    def test_out_of_range_scans_do_not_refresh_or_clear(self):
        self.stop.on_scan(self.scan([1.0]))
        initial_scan = self.stop.last_scan_time
        for _ in range(12):
            self.now += 0.25
            self.stop.on_scan(self.scan([float("nan"), 13.0]))
            self.stop.on_timer(None)
            self.assertTrue(self.stop.active)
        self.assertEqual(self.stop.last_scan_time, initial_scan)
        self.assertIsNone(self.stop.clear_since)

    def test_invalid_scans_cause_timeout_after_clear(self):
        self.clear_stop()
        last_scan = self.stop.last_scan_time
        self.now += 0.2
        self.stop.on_scan(self.scan([float("nan"), -float("inf"), 12.01]))
        self.stop.on_timer(None)
        self.assertTrue(self.stop.active)
        self.assertEqual(self.stop.last_scan_time, last_scan)

    def test_invalid_range_limits_never_refresh_scan(self):
        cases = [
            self.scan([]),
            self.scan([1.0], minimum=float("nan")),
            self.scan([1.0], maximum=float("inf")),
            self.scan([1.0], minimum=-1.0),
            self.scan([1.0], minimum=1.0, maximum=1.0),
            self.scan([1.0], minimum=2.0, maximum=1.0),
        ]
        for scan in cases:
            self.stop.on_scan(scan)
            self.assertIsNone(self.stop.last_scan_time)
            self.assertIsNone(self.stop.clear_since)

    def test_in_range_and_positive_infinity_can_clear(self):
        for reading in (1.0, 12.0, float("inf")):
            with self.subTest(reading=reading):
                self.stop = stop_module.Ld19SafetyStop()
                self.clear_stop(reading)

    def test_close_readings_still_stop_with_invalid_values_present(self):
        for reading in (0.01, 0.02, 0.65):
            with self.subTest(reading=reading):
                self.stop = stop_module.Ld19SafetyStop()
                self.clear_stop()
                self.stop.on_scan(self.scan([13.0, float("nan"), reading]))
                self.assertTrue(self.stop.active)
                self.assertIsNone(self.stop.clear_since)

    def test_scan_gap_restarts_clear_time_even_without_timer(self):
        self.stop.on_scan(self.scan([1.0]))
        self.now += 3.0
        self.stop.on_scan(self.scan([1.0]))
        self.stop.on_timer(None)
        self.assertTrue(self.stop.active)
        self.assertEqual(self.stop.clear_since, rospy.Time.from_sec(self.now))


if __name__ == "__main__":
    unittest.main()
