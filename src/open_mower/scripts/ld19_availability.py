#!/usr/bin/env python

import os
import sys
import time

import roslib.packages
import rospy


def main():
    driver_args = sys.argv[1:]
    rospy.init_node("ld19_availability")
    port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")

    while not rospy.is_shutdown():
        if os.path.exists(port_name) and os.access(port_name, os.R_OK | os.W_OK):
            driver_paths = roslib.packages.find_node("ldlidar_stl_ros", "ldlidar_stl_ros_node")
            if not driver_paths:
                rospy.logerr("Could not find the ldlidar_stl_ros_node executable")
                return 1
            rospy.signal_shutdown("LD19 serial device detected")
            os.execv(driver_paths[0], [driver_paths[0]] + driver_args)

        rospy.loginfo_throttle(30.0, "Waiting for LD19 serial device at %s", port_name)
        time.sleep(1.0)

    return 0


if __name__ == "__main__":
    sys.exit(main())
