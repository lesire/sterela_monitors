#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import sin, cos

class Monitor:
    def __init__(self):
        
        # Here initialize zone data, e.g.:
        self.zone1 = False
        self.zone2 = False
    
        # Connection to ROS topics    
        self.laser_low_subscriber = rospy.Subscriber("/robot/robot_laser", LaserScan, self.laser_low_cb)
        self.laser_high_subscriber = rospy.Subscriber("/robot/pac_laser", LaserScan, self.laser_high_cb)
        self.command_subscriber = rospy.Subscriber("/safe_pilot/command", Twist, self.command_cb)
        self.command_publisher = rospy.Publisher("/robot/control", Twist, queue_size=10)
        
    def laser_low_cb(self, scan):
        # Here check the containt of the low laser scan and set the zone obstacle value accordingly
        
        # Reinit zones
        self.zone1 = False

        
        # Loop over the scan echos
        for i in range(len(scan.ranges)):
            distance = scan.ranges[i]
            # If range has inconsistent values, discard it
            if distance < scan.range_min or distance > scan.range_max:
                continue
            
            # else, compute angle, delta x, delta y
            angle = scan.angle_min + (scan.angle_max - scan.angle_min) * i / len(scan.ranges)
            dy = distance * sin(angle)
            dx = distance * cos(angle)
            
            # now you should probably test dx and dy to define if the zones are occupied by an obstacle, e.g.:
            if dx > 0:
                self.zone1 = True
            if dy > 0 and dy < 10:
                self.zone2 = True 

        rospy.loginfo("Zone 1: " + str(self.zone1))
    
    def laser_high_cb(self, scan):
        # Here check the containt of the high laser scan and set the zone obstacle value accordingly
        # see laser_low_cb
        pass
    
    def command_cb(self, cmd):
        # Use the status of zones to decide, e.g.
        if self.zone1:
            # Send stop!
            self.command_publisher.publish(Twist())
        else:
            # Do nothing, i.e. forward the real Command
            self.command_publisher.publish(cmd)
            
if __name__ == "__main__":
    rospy.init_node("monitor")
    monitor = Monitor()
    rospy.spin()

