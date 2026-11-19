#!/usr/bin/env python3
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Ld19SafetyGate:
    def __init__(self):
        self.timeout = rospy.get_param("~heartbeat_timeout", 0.5)
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            raise ValueError("LD19 safety heartbeat timeout must be positive and finite")

        self.mutex = threading.Lock()
        self.last_heartbeat = None
        self.sensor_locked = True
        self.shutting_down = False
        self.reported_stop = None
        self.velocity_publishers = [
            rospy.Publisher("~nav_vel_out", Twist, queue_size=1),
            rospy.Publisher("~logic_vel_out", Twist, queue_size=1),
        ]
        self.stop_publisher = rospy.Publisher("/ld19_safety/stop", Bool, queue_size=1, latch=True)
        self.lock_subscriber = rospy.Subscriber("/ld19_safety/lock", Bool, self.on_lock, queue_size=1)
        self.velocity_subscribers = [
            rospy.Subscriber("~nav_vel", Twist, self.on_velocity, callback_args=self.velocity_publishers[0], queue_size=1),
            rospy.Subscriber("~logic_vel", Twist, self.on_velocity, callback_args=self.velocity_publishers[1], queue_size=1),
        ]
        self.check()

    def is_stopped(self):
        return (
            self.shutting_down
            or self.sensor_locked
            or self.last_heartbeat is None
            or time.monotonic() - self.last_heartbeat >= self.timeout
        )

    def report(self, stopped):
        if stopped != self.reported_stop:
            self.stop_publisher.publish(Bool(data=stopped))
            if stopped:
                rospy.logwarn("LD19 safety gate stopped: obstacle, stale scan, or missing heartbeat")
            else:
                rospy.loginfo("LD19 safety gate cleared")
            self.reported_stop = stopped

    def on_lock(self, msg):
        with self.mutex:
            self.last_heartbeat = time.monotonic()
            self.sensor_locked = msg.data
            stopped = self.is_stopped()
            if stopped:
                self.stop_automatic_motion()
            self.report(stopped)

    def stop_automatic_motion(self):
        for publisher in self.velocity_publishers:
            publisher.publish(Twist())

    def on_velocity(self, msg, publisher):
        with self.mutex:
            stopped = self.is_stopped()
            publisher.publish(Twist() if stopped else msg)
            self.report(stopped)

    def check(self):
        with self.mutex:
            stopped = self.is_stopped()
            if stopped:
                self.stop_automatic_motion()
            self.report(stopped)

    def shutdown(self):
        with self.mutex:
            self.shutting_down = True
            self.stop_automatic_motion()
            self.report(True)


if __name__ == "__main__":
    rospy.init_node("ld19_safety_gate")
    gate = Ld19SafetyGate()
    rospy.on_shutdown(gate.shutdown)
    while not rospy.is_shutdown():
        gate.check()
        time.sleep(0.05)
