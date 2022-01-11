#!/usr/bin/env python
# import __future__

import rospy
import math
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
pi = math.pi

class pose():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        # self.theta = theta

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
        self.landing_pub = rospy.Publisher('/tello/land', Empty, queue_size=10)

        self.leftcup_pos = pose(0,0)
        self.rightcup_pos = pose(0,0)

        self.leftcup_target = pose(0,0)
        self.rightcup_target = pose(0,0)
        self.k_theta = 0.5
        self.linear_velocity = 0.3
        self.stop_sign = 0
        self.angular_saturation = 0.3

    def start(self):
        self.takeoff()
        Emergency = None
        while Emergency == None and not rospy.is_shutdown():
            Emergency = raw_input("Emergency Stop (Press Enter) ")
            if self.stop_sign == 0:
                self.controller(self.leftcup_pos, self.rightcup_pos)
            else:
                print("Drone landing now !")
                self.vel_publish(0,0)
                self.landing()
        print("Drone landing now !")
        self.vel_publish(0,0)
        self.landing()

    def controller(self, leftcup_pos, rightcup_pos):
        if leftcup_pos == None:
            left_err = 0
        else:
            left_err = self.leftcup_target.x - self.leftcup_pos.x
        
        if rightcup_pos == None:
            right_err = 0
        else:
            right_err = self.rightcup_target.x - self.rightcup_pos.x
        angular_vel = self.k_theta * -left_err + self.k_theta * -right_err
        self.vel_publish(self.linear_velocity, angular_vel)

    def takeoff(self):
        Ready = None
        while Ready == None:
            Ready = raw_input("Ready to take off (Press Enter) ")
            pass
        print("Drone take off now !")
        print("-------------------------")
        rate = rospy.Rate(10)
        rate.sleep()
        self.takeoff_pub.publish(Empty())
        
    def landing(self):
        rate = rospy.Rate(10)
        rate.sleep()
        self.landing_pub.publish(Empty())
        rate.sleep()
    
    def vel_publish(self, vx, w):
        if abs(w) > self.angular_saturation:
            w = self.angular_saturation * w/abs(w)
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = w
        # print("current v, w ="), v, w
        self.vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous = True)
    rate = rospy.Rate(10)
    path_tracker = pathTracker()
    path_tracker.start()
    rospy.spin()
