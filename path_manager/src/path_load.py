#!/usr/bin/env python
# license removed for brevity
from __future__ import division

import numpy as np
import math

import rospy
import tf as trans

from random import *

from std_msgs.msg import Float32MultiArray, Float32, UInt16, UInt8, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

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
    rospy.init_node('path_load', anonymous=True)

    #rospy.Subscriber("/scout/mavros/local_position/pose", PoseStamped, ros.pose_callback, queue_size=2)
    pub_path = rospy.Publisher("/recent_path", Marker, queue_size=1)
    pub_global_path = rospy.Publisher('/global_wp', Float32MultiArray, queue_size=2)

    rate = rospy.Rate(1)  # 20hz

    path = [0.0, 0.0, 0.0]

    pos_x = []
    pos_y = []
    pos_z = []

    f = open('/home/usrg/catkin_ws/src/AI_Challenge/path_manager/src/recent_path.txt', 'r')
    data = f.read()
    f.close()

    for line in data.split('\n'):
        var = line.split()
        if not var:
            break
        pos_x.append(float(var[0]))
        pos_y.append(float(var[1]))
        pos_z.append(float(var[2]))

    marker_path = Marker()

    global_path = Float32MultiArray()

    global_path.layout.data_offset = len(pos_x)
    del global_path.data[:]  # clear msg
    for ind in range(len(pos_x)):
        global_path.data.append(pos_x[ind])
        global_path.data.append(pos_y[ind])
        global_path.data.append(pos_z[ind])

    while not rospy.is_shutdown():

        print(len(pos_x), len(pos_y), len(pos_z))

        marker_path.header = Header(frame_id='scout/map')
        marker_path.ns = "recent path"
        marker_path.type = Marker.SPHERE_LIST
        marker_path.pose.orientation.w = 1.0
        marker_path.scale.x = marker_path.scale.y = marker_path.scale.z = 0.1
        marker_path.color.r = 0.5
        marker_path.color.g = 0.5
        marker_path.color.b = 0.5
        marker_path.color.a = 1.0
        marker_path.id = 0

        for ind in range(len(pos_x)):
            p1 = Point()
            p1.x = pos_x[ind]
            p1.y = pos_y[ind]
            p1.z = pos_z[ind]
            marker_path.points.append(p1)

        pub_path.publish(marker_path)
        pub_global_path.publish(global_path)
        marker_path.points = []

        ros.count = ros.count + 1
        ros.t_cur = ros.count / 20.0
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
