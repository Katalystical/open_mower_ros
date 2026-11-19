#!/usr/bin/env python3
import importlib.util
import os
import unittest
from unittest.mock import patch

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


script = os.path.join(os.path.dirname(__file__), "../scripts/ld19_safety_gate.py")
spec = importlib.util.spec_from_file_location("ld19_safety_gate", script)
gate_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate_module)


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SafetyGateTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.patches = [
            patch.object(gate_module.time, "monotonic", lambda: self.now),
            patch.object(gate_module.rospy, "Publisher", Publisher),
            patch.object(gate_module.rospy, "Subscriber"),
            patch.object(gate_module.rospy, "get_param", lambda name, default: default),
            patch.object(gate_module.rospy, "logwarn"),
            patch.object(gate_module.rospy, "loginfo"),
        ]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.gate = gate_module.Ld19SafetyGate()
        self.command = Twist()
        self.command.linear.x = 0.3
        self.command.angular.z = 0.2

    def assert_stopped(self):
        for publisher in self.gate.velocity_publishers:
            command = publisher.messages[-1]
            self.assertEqual(command.linear.x, 0.0)
            self.assertEqual(command.angular.z, 0.0)
        self.assertTrue(self.gate.stop_publisher.messages[-1].data)

    def test_startup_is_stopped_until_live_clear_heartbeat(self):
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.assert_stopped()
        self.gate.on_lock(Bool(data=False))
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
            self.assertIs(publisher.messages[-1], self.command)
        self.assertFalse(self.gate.stop_publisher.messages[-1].data)

    def test_lock_immediately_publishes_zero_and_blocks_automatic_commands(self):
        self.gate.on_lock(Bool(data=False))
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.gate.on_lock(Bool(data=True))
        self.assert_stopped()
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.assert_stopped()
        self.gate.check()
        self.assert_stopped()

    def test_missing_heartbeat_stops_and_recovers_without_replaying_commands(self):
        self.gate.on_lock(Bool(data=False))
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.now += 0.5
        self.gate.check()
        self.assert_stopped()
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.assert_stopped()
        self.gate.on_lock(Bool(data=False))
        self.assertFalse(self.gate.stop_publisher.messages[-1].data)
        for publisher in self.gate.velocity_publishers:
            self.assertEqual(publisher.messages[-1].linear.x, 0.0)
            self.gate.on_velocity(self.command, publisher)
            self.assertIs(publisher.messages[-1], self.command)

    def test_velocity_cannot_slip_through_before_timeout_check(self):
        self.gate.on_lock(Bool(data=False))
        self.now += 0.5
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.assert_stopped()

    def test_shutdown_stops_even_after_clear_heartbeat(self):
        self.gate.on_lock(Bool(data=False))
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.gate.shutdown()
        self.assert_stopped()
        self.gate.on_lock(Bool(data=False))
        for publisher in self.gate.velocity_publishers:
            self.gate.on_velocity(self.command, publisher)
        self.assert_stopped()


if __name__ == "__main__":
    unittest.main()
