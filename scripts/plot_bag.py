#!/usr/bin/env python
import rospy, rosbag
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64
import sys

bag = rosbag.Bag(sys.argv[1])

xs = {'timestamps': [], 'values': []}
distances = {'timestamps': [], 'values': []}

for topic, msg, t in bag.read_messages():
    if topic == "/robot/pose":
        xs['timestamps'].append(t.to_sec())
        xs['values'].append(msg.pose.position.x)
        
    if topic == "/monitor/distance":
        distances['timestamps'].append(t.to_sec())
        distances['values'].append(msg.data)
        
import matplotlib.pyplot as plt
plt.plot(xs['timestamps'], xs['values'], label="x")
plt.plot(distances['timestamps'], distances['values'], label="obstacle distance")
plt.plot(distances['timestamps'], [0.4]*len(distances['timestamps']), 'r-', label="safe limit")
plt.legend()
plt.savefig(sys.argv[2] + ".png")
plt.ylim(-1,1)
plt.savefig(sys.argv[2] + "_zoom.png")
