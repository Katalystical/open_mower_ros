#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafetyGateIntegrationTest(unittest.TestCase):
    def test_mux_commands_and_heartbeat(self):
        output = []
        stops = []
        output_sub = rospy.Subscriber("/ll/cmd_vel", Twist, lambda msg: output.append(msg.linear.x), queue_size=100)
        stop_sub = rospy.Subscriber("/ld19_safety/stop", Bool, lambda msg: stops.append(msg.data), queue_size=100)
        heartbeat = rospy.Publisher("/ld19_safety/lock", Bool, queue_size=1)
        inputs = {
            name: rospy.Publisher("/" + name, Twist, queue_size=1)
            for name in ("nav_vel", "logic_vel", "manual_vel", "override_vel")
        }
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if heartbeat.get_num_connections() and all(pub.get_num_connections() for pub in inputs.values()):
                break
            time.sleep(0.05)
        else:
            self.fail("ROS subscriptions did not connect")

        command = Twist()
        command.linear.x = 0.3
        inputs["nav_vel"].publish(command)
        time.sleep(0.2)
        self.assertTrue(output and output[-1] == 0.0 and stops and stops[-1])

        for name in ("override_vel", "manual_vel", "logic_vel", "nav_vel"):
            for _ in range(15):
                heartbeat.publish(Bool(data=False))
                time.sleep(0.05)
            output.clear()
            for _ in range(5):
                heartbeat.publish(Bool(data=False))
                inputs[name].publish(command)
                time.sleep(0.05)
            self.assertTrue(any(abs(value - 0.3) < 1e-6 for value in output), name)

        for _ in range(5):
            heartbeat.publish(Bool(data=False))
            inputs["manual_vel"].publish(command)
            time.sleep(0.05)
        output.clear()
        for _ in range(5):
            heartbeat.publish(Bool(data=True))
            inputs["manual_vel"].publish(command)
            time.sleep(0.05)
        self.assertTrue(output)
        self.assertTrue(all(abs(value - 0.3) < 1e-6 for value in output))

        for _ in range(15):
            heartbeat.publish(Bool(data=False))
            time.sleep(0.05)
        inputs["nav_vel"].publish(command)
        time.sleep(0.1)
        self.assertAlmostEqual(output[-1], 0.3)
        output.clear()
        heartbeat.publish(Bool(data=True))
        time.sleep(0.2)
        self.assertTrue(output and output[-1] == 0.0 and stops[-1])
        for name in ("nav_vel", "logic_vel"):
            inputs[name].publish(command)
            time.sleep(0.1)
            self.assertEqual(output[-1], 0.0)
        for name in ("override_vel", "manual_vel"):
            for _ in range(15):
                heartbeat.publish(Bool(data=True))
                time.sleep(0.05)
            inputs[name].publish(command)
            time.sleep(0.1)
            self.assertAlmostEqual(output[-1], 0.3, msg=name)

        for _ in range(15):
            heartbeat.publish(Bool(data=False))
            time.sleep(0.05)
        inputs["nav_vel"].publish(command)
        time.sleep(0.1)
        self.assertAlmostEqual(output[-1], 0.3)
        self.assertFalse(stops[-1])
        output.clear()
        time.sleep(0.7)
        self.assertTrue(output and output[-1] == 0.0 and stops[-1])
        for name in ("nav_vel", "logic_vel"):
            inputs[name].publish(command)
            time.sleep(0.1)
            self.assertEqual(output[-1], 0.0)
        for name in ("override_vel", "manual_vel"):
            time.sleep(0.7)
            inputs[name].publish(command)
            time.sleep(0.1)
            self.assertAlmostEqual(output[-1], 0.3, msg=name)
        heartbeat.publish(Bool(data=False))
        time.sleep(0.1)
        output.clear()
        for _ in range(16):
            inputs["manual_vel"].publish(command)
            time.sleep(0.05)
        self.assertTrue(stops[-1])
        self.assertTrue(output)
        self.assertTrue(all(abs(value - 0.3) < 1e-6 for value in output))
        output_sub.unregister()
        stop_sub.unregister()


if __name__ == "__main__":
    rospy.init_node("ld19_safety_gate_integration")
    rostest.rosrun("open_mower", "ld19_safety_gate_integration", SafetyGateIntegrationTest)
