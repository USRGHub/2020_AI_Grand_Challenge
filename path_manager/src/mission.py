#!/usr/bin/env python
# license removed for brevity
from __future__ import division

import numpy as np
import math

import rospy
import tf as trans

from random import *

from std_msgs.msg import Float32MultiArray, Float32, UInt16, UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


eps = 0.0000001


class ros_class:
    def __init__(self):
        self.t_cur = 0.0
        self.count = 0
        self.done = False

        self.pub_GoalAction_uav1 = 0
        self.pub_GoalAction_uav2 = 0
        self.pub_VelCmd = 0
        self.Cur_Pos_m = [0.0, 0.0, 0.0]
        self.Cur_Vel_mps = [0.0, 0.0, 0.0]
        self.init_x = [0.0, 0.0, 0.0]

        self.Tar_Pos_m = [0.0, 0.0, 0.0]
        self.Tar_Vel_mps = [0.0, 0.0, 0.0]
        self.euler_tar = [0.0, 0.0, 0.0]

        self.q = []
        self.q_tar = []
        self.euler = [0.0, 0.0, 0.0]

        self.init_uav1 = [0.0, 0.0, 4.0]
        self.init_uav2 = [15.0, 0.0, 2.0]
        self.init = [0.0, 0.0, 2.0]

        self.rand = [0.0, 0.0]

        self.cmd = [0.0, 0.0, 0.0, 0.0]
        self.t_capt = 0.0
        self.t_local = 0.0

        self.image_pos = [0.0, 0.0]
        self.u = 0
        self.v = 0
        self.Cur_Tar_m = [0.0, 0.0]
        self.GoalAction_uav1 = Float32MultiArray()
        self.GoalAction_uav2 = Float32MultiArray()

    def reset(self):
        #angle = np.arctan2(random()-0.5, random()-0.5)
        #self.init[0] = 1.0*np.cos(angle)
        #self.init[1] = 1.0*np.sin(angle)
        #self.init[2] = (random() + 1.0)

        # Case 1.
        self.init[0] = 0.0
        self.init[1] = 0.0
        self.init[2] = 2.0

        self.N_ballon = 5
        self.mission = 6

        self.t_cur = 0.0
        self.count = 0

    def odom_callback(self, data):
        self.Cur_Pos_m[0] = data.pose.pose.position.x
        self.Cur_Pos_m[1] = data.pose.pose.position.y
        self.Cur_Pos_m[2] = data.pose.pose.position.z

        self.q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,)

        self.euler = trans.transformations.euler_from_quaternion(self.q)

    def tar_odom_callback(self, data):
        self.Tar_Pos_m[0] = data.pose.pose.position.x
        self.Tar_Pos_m[1] = data.pose.pose.position.y
        self.Tar_Pos_m[2] = data.pose.pose.position.z

        self.q_tar = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,)

        self.euler_tar = trans.transformations.euler_from_quaternion(self.q_tar)

    def vel_callback(self, data):
        self.Cur_Vel_mps[0] = data.twist.linear.x
        self.Cur_Vel_mps[1] = data.twist.linear.y
        self.Cur_Vel_mps[2] = data.twist.linear.z

    def tar_vel_callback(self, data):
        self.Tar_Vel_mps[0] = data.twist.linear.x
        self.Tar_Vel_mps[1] = data.twist.linear.y
        self.Tar_Vel_mps[2] = data.twist.linear.z

    def mission_flow(self):
        if self.mission == 1:
            ## Mission Start & Take-off
            a = 0

        if self.mission == 2:
            a = 0

        if self.mission == 3:
            ## Tracking Mode
            a = 0

        if self.mission == 4:
            ## Landing Waypoint
            a = 0

        if self.mission == 5:
            ## Landing
            a = 0


def main():
    ros = ros_class()
    rospy.init_node('mission_tracking', anonymous=True)

    rospy.Subscriber("/uav1/odom", Odometry, ros.odom_callback, queue_size=2)
    rospy.Subscriber("/uav1/mavros/local_position/velocity_local", TwistStamped, ros.vel_callback, queue_size=2)
    ros.pub_GoalAction_uav1 = rospy.Publisher("/uav1/GoalAction", Float32MultiArray, queue_size=2)

    rospy.Subscriber("/uav2/odom", Odometry, ros.tar_odom_callback, queue_size=2)
    rospy.Subscriber("/uav2/mavros/local_position/velocity_local", TwistStamped, ros.tar_vel_callback, queue_size=2)
    ros.pub_GoalAction_uav2 = rospy.Publisher("/uav2/GoalAction", Float32MultiArray, queue_size=2)

    rate = rospy.Rate(20)  # 20hz

    ros.reset()
    flag = 0
    diff = [100, 100, 100]

    while not rospy.is_shutdown():

        if (ros.t_cur % 1.0) == 0:
            #print("[time] %.3f [mission] %d [WP index] %d" % (ros.t_cur, ros.mission, ros.WP_index))
            print("[time] %.3f [mission] %d" % (ros.t_cur, flag))
            #print("[flag] %d [cur]  %.3f  %.3f  %.3f" % (ros.flag, ros.Cur_Pos_m[0], ros.Cur_Pos_m[1], ros.Cur_Pos_m[2]))
            print("\n\n\n")

            if flag == 0:
                if ros.t_cur > 2.0 or ros.Cur_Pos_m[2] > 1.0:
                    flag = 1
                else:
                    ## UAV1 initialize
                    ros.GoalAction_uav1.data.append(0)  # Mission
                    ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
                    ros.GoalAction_uav1.data = []

                    ## UAV2 initialize
                    ros.GoalAction_uav2.data.append(0)  # Mission
                    ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
                    ros.GoalAction_uav2.data = []

            if flag == 1:

                if (ros.init_uav1[2] - ros.Cur_Pos_m[2]) < 0.1:
                    flag = 2
                else:
                    ros.GoalAction_uav1.data.append(1)  # Mission
                    ros.GoalAction_uav1.data.append(ros.init_uav1[0])  # init x
                    ros.GoalAction_uav1.data.append(ros.init_uav1[1])  # init y
                    ros.GoalAction_uav1.data.append(ros.init_uav1[2])  # init z
                    ros.GoalAction_uav1.data.append(0.0)  # init r
                    ros.GoalAction_uav1.data.append(1.0)  # init vx
                    ros.GoalAction_uav1.data.append(1.5)  # init vz

                    ros.GoalAction_uav2.data.append(1)  # Mission
                    ros.GoalAction_uav2.data.append(ros.init_uav2[0])  # init x
                    ros.GoalAction_uav2.data.append(ros.init_uav2[1])  # init y
                    ros.GoalAction_uav2.data.append(ros.init_uav2[2])  # init z
                    ros.GoalAction_uav2.data.append(0.0)  # init r
                    ros.GoalAction_uav2.data.append(1.0)  # init vx
                    ros.GoalAction_uav2.data.append(1.5)  # init vz

                    ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
                    ros.GoalAction_uav1.data = []

                    ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
                    ros.GoalAction_uav2.data = []

            if flag == 2:
                ros.GoalAction_uav1.data.append(3)             # Mission
                ros.GoalAction_uav1.data.append(ros.init_uav1[0])  # init x
                ros.GoalAction_uav1.data.append(ros.init_uav1[1])  # init y
                ros.GoalAction_uav1.data.append(ros.init_uav1[2])  # init z
                ros.GoalAction_uav1.data.append(0.0)           # init r
                ros.GoalAction_uav1.data.append(4.0)           # init vx
                ros.GoalAction_uav1.data.append(1.0)           # init vz
                ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
                ros.GoalAction_uav1.data = []

                diff[0] = ros.init_uav1[0] - ros.Cur_Pos_m[0]
                diff[1] = ros.init_uav1[1] - ros.Cur_Pos_m[1]
                diff[2] = ros.init_uav1[2] - ros.Cur_Pos_m[2]

                dist = np.sqrt(diff[0]*diff[0] + diff[1]*diff[1])

                ros.GoalAction_uav2.data.append(3)  # Mission
                ros.GoalAction_uav2.data.append(ros.init_uav2[0])  # init x
                ros.GoalAction_uav2.data.append(ros.init_uav2[1])  # init y
                ros.GoalAction_uav2.data.append(ros.init_uav2[2])  # init z
                ros.GoalAction_uav2.data.append(0.0)  # init r
                ros.GoalAction_uav2.data.append(4.0)  # init vx
                ros.GoalAction_uav2.data.append(1.5)  # init vz
                ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
                ros.GoalAction_uav2.data = []

                diff[0] = ros.init_uav2[0] - ros.Tar_Pos_m[0]
                diff[1] = ros.init_uav2[1] - ros.Tar_Pos_m[1]
                diff[2] = ros.init_uav2[2] - ros.Tar_Pos_m[2]

                dist_2 = np.sqrt(diff[0]*diff[0] + diff[1]*diff[1])

                if dist < 0.5 and dist_2 < 1.0:
                    ros.t_capt = ros.t_cur
                    flag = 3

            if flag == 3:
                ros.t_local = ros.t_cur - ros.t_capt
                #print("mission start!")
                ros.GoalAction_uav1.data.append(6)  # Mission
                ros.GoalAction_uav1.data.append(0.0)  # init x
                ros.GoalAction_uav1.data.append(0.0)  # init y
                ros.GoalAction_uav1.data.append(4.0)  # init z
                ros.GoalAction_uav1.data.append(0.0)  # init r
                ros.GoalAction_uav1.data.append(4.0)  # init vx
                ros.GoalAction_uav1.data.append(1.5)  # init vz
                ros.pub_GoalAction_uav1.publish(ros.GoalAction_uav1)
                ros.GoalAction_uav1.data = []

                ros.GoalAction_uav2.data.append(6)  # Mission
                ros.GoalAction_uav2.data.append(ros.init_uav2[0])  # init x
                ros.GoalAction_uav2.data.append(ros.init_uav2[1])  # init y
                ros.GoalAction_uav2.data.append(ros.init_uav2[2])  # init z

                if ros.t_local < 3.0:
                    ros.GoalAction_uav2.data.append(0.0)  # init r
                else:
                    ros.GoalAction_uav2.data.append(np.pi/2.0)  # init r

                ros.GoalAction_uav2.data.append(3.0)  # init vx
                ros.GoalAction_uav2.data.append(1.5)  # init vz
                ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
                ros.GoalAction_uav2.data = []

                diff[0] = ros.Cur_Pos_m[0] - ros.Tar_Pos_m[0]
                diff[1] = ros.Cur_Pos_m[1] - ros.Tar_Pos_m[1]
                diff[2] = ros.Cur_Pos_m[2] - ros.Tar_Pos_m[2]

                dist_3 = np.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

                if dist_3 < 1.5:
                    flag = 2

        ros.count = ros.count + 1
        ros.t_cur = ros.count / 20.0
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
