#!/usr/bin/env python
# license removed for brevity
from __future__ import division

import numpy as np
import math

import rospy
import tf as trans

from random import *

from std_msgs.msg import Float32MultiArray, UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


class ros_class:
    def __init__(self):
        self.t_cur = 0.0
        self.count = 0
        self.done = False

        self.pub_GoalAction_uav1 = 0
        self.pub_GoalAction_target = 0
        self.pub_VelCmd = 0
        self.Cur_Pos_m = [0.0, 0.0, 0.0]
        self.Cur_Vel_mps = [0.0, 0.0, 0.0]
        self.init_x = [0.0, 0.0, 0.0]
        self.q = []
        self.euler = [0.0, 0.0, 0.0]
        self.init = [0.0, 0.0, 0.0]

        self.cmd = [0.0, 0.0, 0.0, 0.0]

        self.image_pos = [0.0, 0.0]
        self.u = 0
        self.v = 0
        self.Cur_Tar_m = [0.0, 0.0]
        self.GoalAction_uav1 = Float32MultiArray()
        self.GoalAction_target = Float32MultiArray()

        self.flag_uav1 = 0
        self.flag_target = 0
        self.PubMissionCallback = 0

    def reset(self):
        #angle = np.arctan2(random()-0.5, random()-0.5)
        #self.init[0] = 1.0*np.cos(angle)
        #self.init[1] = 1.0*np.sin(angle)
        #self.init[2] = (random() + 1.0)

        # Case 1.
        self.init[0] = 0.0
        self.init[1] = 0.0
        self.init[2] = 2.0

        print(self.init[0], self.init[1])

        if self.Cur_Pos_m[2] > 0.5:
            self.flag = 1
        else:
            self.flag = 0

        self.t_cur = 0.0
        self.count = 0

    def uav1_odom_callback(self, data):

        self.Cur_Pos_m[0] = data.pose.pose.position.x
        self.Cur_Pos_m[1] = data.pose.pose.position.y
        self.Cur_Pos_m[2] = data.pose.pose.position.z

        self.q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,)

        self.euler = trans.transformations.euler_from_quaternion(self.q)

    def uav1_mission_callback(self, data):
        self.flag_uav1 = data.data

    def target_mission_callback(self, data):
        self.flag_target = data.data


def main():
    ros = ros_class()
    rospy.init_node('uav_tracking_mission', anonymous=True)

    rospy.Subscriber("/uav1/odom", Odometry, ros.uav1_odom_callback, queue_size=2)

    rospy.Subscriber("/uav1/mission", UInt8, ros.uav1_mission_callback, queue_size=1)
    ros.pub_GoalAction_uav1 = rospy.Publisher("/uav1/GoalAction", Float32MultiArray, queue_size=10)

    rospy.Subscriber("/target/mission", UInt8, ros.target_mission_callback, queue_size=1)
    ros.pub_GoalAction_target = rospy.Publisher("/target/GoalAction", Float32MultiArray, queue_size=10)

    rate = rospy.Rate(20)  # 20hz

    ros.reset()

    while not rospy.is_shutdown():

        if (ros.t_cur % 1.0) == 0:
            print("[time] %.3f [init] %.3f  %.3f  %.3f" % (ros.t_cur, ros.init[0], ros.init[1], ros.init[2]))
            print("[flag] %d [cur]  %.3f  %.3f  %.3f" % (ros.flag, ros.Cur_Pos_m[0], ros.Cur_Pos_m[1], ros.Cur_Pos_m[2]))
            print("\n\n\n")

        #
        # Init --> Disarm and Ready to Arming
        #
        if ros.flag_uav1 == 0:
            ros.GoalAction_uav1.data.append(0)  # Mission
            ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
            ros.GoalAction_uav1.data = []
            ros.PubMissionCallback = 1
            #if ros.t_cur > 2.0:
            #    ros.flag = 1

        if ros.flag_target == 0:
            ros.GoalAction_target.data.append(0)  # Mission
            ros.pub_GoalAction_target.publish(ros.GoalAction_target)
            ros.GoalAction_target.data = []

        #
        # takeoff
        #
        if ros.flag_uav1 == 1:
            ros.GoalAction_uav1.data.append(1)  # Mission
            ros.GoalAction_uav1.data.append(30.0) # init x
            ros.GoalAction_uav1.data.append(50.0) # init y
            ros.GoalAction_uav1.data.append(2.0)  # init z
            ros.GoalAction_uav1.data.append(3.14)  # init r
            ros.GoalAction_uav1.data.append(1.0)  # init vx
            ros.GoalAction_uav1.data.append(1.5)  # init vz
            ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
            ros.GoalAction_uav1.data = []

        if ros.flag_target == 1:
            ros.GoalAction_target.data.append(1)  # Mission
            ros.GoalAction_target.data.append(20.0)  # init x
            ros.GoalAction_target.data.append(50.0)  # init y
            ros.GoalAction_target.data.append(10.0)  # init z
            ros.GoalAction_target.data.append(0.0)  # init r
            ros.GoalAction_target.data.append(1.0)  # init vx
            ros.GoalAction_target.data.append(1.5)  # init vz
            ros.pub_GoalAction_target.publish(ros.GoalAction_target)
            ros.GoalAction_target.data = []

        #
        # waypoint
        #
        if ros.flag_uav1 == 2:
            ros.GoalAction_uav1.data.append(3)  # Mission
            ros.GoalAction_uav1.data.append(30.0)  # init x
            ros.GoalAction_uav1.data.append(50.0)  # init y
            ros.GoalAction_uav1.data.append(2.0)  # init z
            ros.GoalAction_uav1.data.append(3.14)  # init r
            ros.GoalAction_uav1.data.append(2.0)  # init vx
            ros.GoalAction_uav1.data.append(1.0)  # init vz
            ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
            ros.GoalAction_uav1.data = []

        if ros.flag_target == 2:
            ros.GoalAction_target.data.append(3)  # Mission
            ros.GoalAction_target.data.append(20.0)  # init x
            ros.GoalAction_target.data.append(50.0)  # init y
            ros.GoalAction_target.data.append(10.0)  # init z
            ros.GoalAction_target.data.append(0.0)  # init r
            ros.GoalAction_target.data.append(2.0)  # init vx
            ros.GoalAction_target.data.append(1.0)  # init vz
            ros.pub_GoalAction_target.publish(ros.GoalAction_target)
            ros.GoalAction_target.data = []

        #
        # landing
        #
        if ros.flag_uav1 == 3:
            ros.GoalAction_uav1.data.append(2)  # Mission
            ros.GoalAction_uav1.data.append(30.0)  # init x
            ros.GoalAction_uav1.data.append(50.0)  # init y
            ros.GoalAction_uav1.data.append(10.0)  # init z
            ros.GoalAction_uav1.data.append(3.14)  # init r
            ros.GoalAction_uav1.data.append(1.0)  # init vx
            ros.GoalAction_uav1.data.append(-0.5)  # init vz
            ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
            ros.GoalAction_uav1.data = []

        if ros.flag_target == 3:
            ros.GoalAction_target.data.append(2)  # Mission
            ros.GoalAction_target.data.append(20.0)  # init x
            ros.GoalAction_target.data.append(50.0)  # init y
            ros.GoalAction_target.data.append(10.0)  # init z
            ros.GoalAction_target.data.append(0.0)  # init r
            ros.GoalAction_target.data.append(1.0)  # init vx
            ros.GoalAction_target.data.append(-0.5)  # init vz
            ros.pub_GoalAction_target.publish(ros.GoalAction_target)
            ros.GoalAction_target.data = []

        #
        # mission
        #
        if ros.flag_uav1 == 4:
            ros.GoalAction_uav1.data.append(6)  # Mission
            ros.GoalAction_uav1.data.append(0.0)  # init x
            ros.GoalAction_uav1.data.append(0.0)  # init y
            ros.GoalAction_uav1.data.append(0.0)  # init z
            ros.GoalAction_uav1.data.append(0.0)  # init r
            ros.GoalAction_uav1.data.append(1.0)  # init vx
            ros.GoalAction_uav1.data.append(1.0)  # init vz
            ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
            ros.GoalAction_uav1.data = []

        if ros.flag_target == 4:
            ros.GoalAction_target.data.append(6)  # Mission
            ros.GoalAction_target.data.append(0.0)  # init x
            ros.GoalAction_target.data.append(0.0)  # init y
            ros.GoalAction_target.data.append(0.0)  # init z
            ros.GoalAction_target.data.append(0.0)  # init r
            ros.GoalAction_target.data.append(1.0)  # init vx
            ros.GoalAction_target.data.append(1.0)  # init vz
            ros.pub_GoalAction_target.publish(ros.GoalAction_target)
            ros.GoalAction_target.data = []

        if ros.PubMissionCallback == 1:
            ros.count = ros.count + 1
            ros.t_cur = ros.count / 20.0

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
