#!/usr/bin/env python
# import __future__

import rospy
import math
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
pi = math.pi

class pose():
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other): 
            if not isinstance(other, pose):
                # don't attempt to compare against unrelated types
                return NotImplemented
            return self.x == other.x and self.y == other.y

class pathTracker():
    def __init__(self):
        # self.cup_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.poseCallback)
        self.vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
        self.landing_pub = rospy.Publisher('/tello/cmd_vel', Empty, queue_size=10)

    def start(self):
        print("Drone take off now !")
        self.takeoff()
        while not rospy.is_shutdown():
            pass
        print("Drone landing now !")
        self.landing()

    def takeoff(self):
        msg = Empty()
        self.takeoff_pub.publish(msg)

    def landing(self):
        msg = Empty()
        self.landing_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous = True)
    rate = rospy.Rate(10)
    path_tracker = pathTracker()
    path_tracker.start()
    rospy.spin()
