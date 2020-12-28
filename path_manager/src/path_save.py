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

        self.pub_VelCmd = 0
        self.Cur_Pos_m = [0.0, 0.0, 0.0]
        self.Cur_Vel_mps = [0.0, 0.0, 0.0]
        self.init_x = [0.0, 0.0, 0.0]

        self.q = []
        self.euler = [0.0, 0.0, 0.0]

        self.init = [0.0, 0.0, 2.0]

        self.t_capt = 0.0
        self.t_local = 0.0

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

    def pose_callback(self, data):
        self.Cur_Pos_m[0] = data.pose.position.x
        self.Cur_Pos_m[1] = data.pose.position.y
        self.Cur_Pos_m[2] = data.pose.position.z

        self.q = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,)

        self.euler = trans.transformations.euler_from_quaternion(self.q)


def main():
    ros = ros_class()
    rospy.init_node('path_save', anonymous=True)

    rospy.Subscriber("/scout/mavros/local_position/pose", PoseStamped, ros.pose_callback, queue_size=2)

    rate = rospy.Rate(20)  # 20hz

    path = [0.0, 0.0, 0.0]

    while not rospy.is_shutdown():

        dx = path[0] - ros.Cur_Pos_m[0]
        dy = path[1] - ros.Cur_Pos_m[1]
        dz = path[2] - ros.Cur_Pos_m[2]

        dist = np.sqrt(dx*dx + dy*dy + dz*dz)

        if dist > 0.2:
            f = open('/home/usrg/catkin_ws/src/AI_Challenge/path_manager/src/recent_path.txt', 'a')
            f.write(str(ros.Cur_Pos_m[0]) + '  ')
            f.write(str(ros.Cur_Pos_m[1]) + '  ')
            f.write(str(ros.Cur_Pos_m[2]) + '  ')
            f.write('\n')
            f.close()

            path[0] = ros.Cur_Pos_m[0]
            path[1] = ros.Cur_Pos_m[1]
            path[2] = ros.Cur_Pos_m[2]

        ros.count = ros.count + 1
        ros.t_cur = ros.count / 20.0
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
