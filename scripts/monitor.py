#!/usr/bin/env python
from math import sin, cos

import rospy
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Enum for describing the states
class State(Enum):
    SAFE = 2.0
    WARNING = 0.5
    COLLISION = 0.0


class Monitor:
    def __init__(self):
        # State flags
        self.state = State.SAFE

        # Here initialize robot dimension (in meters)
        self.platform_width = 1.0
        self.platform_len = 1.14
        self.arm_len = 1.3

        # Here initialize zone data, e.g.:
        self.zone1 = False
        self.dx1 = 50.0
        self.zone2_high = False
        self.dx2_high = 50.0
        self.zone2_low = False
        self.dx2_low = 50.0

        # Connection to ROS topics    
        self.laser_low_subscriber = rospy.Subscriber("/robot/robot_laser", LaserScan, self.laser_low_cb)
        self.laser_high_subscriber = rospy.Subscriber("/robot/pac_laser", LaserScan, self.laser_high_cb)
        self.command_subscriber = rospy.Subscriber("/safe_pilot/command", Twist, self.command_cb)
        self.command_publisher = rospy.Publisher("/robot/control", Twist, queue_size=10)

    def laser_low_cb(self, scan):
        # Here check the containt of the low laser scan and set the zone obstacle value accordingly

        # Re-init zones
        self.zone1 = False
        self.zone2_low = False
        self.dx1 = 50.0
        self.dx2_low = 50.0


        # Loop over the scan echos
        for i in range(len(scan.ranges)):
            distance = scan.ranges[i]
            # If range has inconsistent values, discard it
            if distance < scan.range_min or distance > 5.0:
                continue

            # else, compute angle, delta x, delta y
            angle = scan.angle_min + (scan.angle_max - scan.angle_min) * i / len(scan.ranges)
            dy = distance * sin(angle)
            dx = distance * cos(angle)

            # Determine the occupied zone
            if -self.platform_width < dy < 0.0:
                self.zone1 = True
                if self.dx1 > dx:
                    self.dx1 = dx
            elif 0.0 < dy < self.arm_len:
                self.zone2_low = True
                if self.dx2_low > dx:
                    self.dx2_low = dx

        rospy.loginfo("Zone 1: " + str(self.zone1))
        rospy.loginfo("Mininum distance in zone 1: " + str(self.dx1))
        rospy.loginfo("Zone 2 low: " + str(self.zone2_low))
        rospy.loginfo("Mininum distance in zone 2 low: " + str(self.dx2_low))

    def laser_high_cb(self, scan):
        # Here check the contains of the low laser scan and set the zone obstacle value accordingly

        # Re-init zones
        self.zone2_high = False
        self.dx2_high = 50.0

        # Loop over the scan echos
        for i in range(len(scan.ranges)):
            distance = scan.ranges[i]
            # If range has inconsistent values, discard it
            if distance < scan.range_min or distance > 5.0:
                continue

            # else, compute angle, delta x, delta y
            angle = scan.angle_min + (scan.angle_max - scan.angle_min) * i / len(scan.ranges)
            dy = distance * sin(angle)
            dx = distance * cos(angle)

            # Determine the occupied zone
            if (-self.arm_len) / 2.0 < dy < self.arm_len / 2.0:
                self.zone2_high = True
                if self.dx2_high > dx:
                    self.dx2_high = dx

        rospy.loginfo("Zone 2 high: " + str(self.zone2_high))
        rospy.loginfo("Mininum distance in zone 2 high: " + str(self.dx2_high))

    def command_cb(self, cmd):
        # Compute the flag
        # dx2 = 50.0
        # if self.zone2_high or self.zone2_low:
        #     dx2 = min(self.dx2_low, self.dx2_high-self.platform_len/2)
        dx2 = self.dx2_high-self.platform_len/2
        if self.dx1 >= State.SAFE and dx2 >= State.SAFE:
            self.state=State.SAFE
        if State.SAFE < self.dx1 < State.WARNING or State.SAFE < dx2 < State.WARNING:
            self.state = State.WARNING
        if State.COLLISION < self.dx1 < State.WARNING or State.COLLISION < dx2 < State.WARNING:
            self.state = State.COLLISION
            
        rospy.loginfo("State: " + str(self.state))
        # Use the status of zones to decide, e.g.
        if self.state == State.WARNING or self.state == State.COLLISION:
            # Send stop!
            if cmd.linear.x < 0.01:
                rospy.loginfo("Stopped by controller")
            else:
                rospy.logwarn("MONITOR ACTIVATED")
            self.command_publisher.publish(Twist())
        else:
            # Do nothing, i.e. forward the real Command
            self.command_publisher.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("monitor")
    monitor = Monitor()
    rospy.spin()
