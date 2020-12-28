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

        self.pub_GoalAction_target = 0

        self.pub_VelCmd = 0
        self.Cur_Pos_m = [0.0, 0.0, 0.0]
        self.Cur_Vel_mps = [0.0, 0.0, 0.0]
        self.init_x = [0.0, 0.0, 0.0]
        self.q = []
        self.euler = [0.0, 0.0, 0.0]
        self.init = [0.0, 0.0, 0.0]

        self.cmd = [0.0, 0.0, 0.0, 0.0]
        self.GoalAction_target = Float32MultiArray()

        self.flag_target = -1
        self.PubMissionCallback = 0

        ## Mission Planning
        self.flag_mission = 0
        self.flag_landing = 0

        self.mission = 2

        self.WP_index = 0
        self.WP_num = 4
        self.WP_dist = 10.0
        self.WP_dir = 0.0
        self.WP_eta = 2.0
        self.WayPoint_X = [-11.591, -6.415, 50.228, 55.405, 10.0, 30.0]
        self.WayPoint_Y = [3.106, 22.424,  -13.459, 5.860, 10.0, 90.0]
        self.WayPoint_Z = [10.0,  10.0,  10.0,  10.0,  2.0,  2.0]
        self.delPos = [0.0, 0.0, 0.0]
        self.PreWayPoint = [0.0, 0.0, 0.0]
        self.TakeoffPoint = [20.0, 50.0, 2.0]
        self.LandPoint = [20.0, 50.0, 2.0]
        self.Init_heading = 1.309
        self.Start_Pos = [10, 50]
        self.Start_Pos_x = 22.907
        self.Start_Pos_y = 4.483

    def reset(self):
        #angle = np.arctan2(random()-0.5, random()-0.5)
        #self.init[0] = 1.0*np.cos(angle)
        #self.init[1] = 1.0*np.sin(angle)
        #self.init[2] = (random() + 1.0)

        # Case 1.
        self.init[0] = 0.0
        self.init[1] = 0.0
        self.init[2] = 2.0

        self.mission = 1

        self.t_cur = 0.0
        self.count = 0

    def target_odom_callback(self, data):

        self.Cur_Pos_m[0] = data.pose.pose.position.x + self.Start_Pos_x
        self.Cur_Pos_m[1] = data.pose.pose.position.y + self.Start_Pos_y
        self.Cur_Pos_m[2] = data.pose.pose.position.z

        self.q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,)

        self.euler = trans.transformations.euler_from_quaternion(self.q)
        self.flag_mission = 1

    def target_mission_callback(self, data):
        self.flag_target = data.data

    def mission_flow(self):
        if self.mission == 1:
            ## Mission Start & Take-off
            self.GoalAction_target.data.append(1)    # Mission
            self.GoalAction_target.data.append(self.Start_Pos_x) # init x
            self.GoalAction_target.data.append(self.Start_Pos_y) # init y
            self.GoalAction_target.data.append(10.0)  # init z
            self.GoalAction_target.data.append(self.Init_heading) # init r
            self.GoalAction_target.data.append(1.0)  # init vx
            self.GoalAction_target.data.append(1.5)  # init vz
            self.pub_GoalAction_target.publish(self.GoalAction_target)
            self.GoalAction_target.data = []
            dist = np.fabs(4.0 - self.Cur_Pos_m[2])
            if dist < 0.2 or  self.Cur_Pos_m[2] > 2.0:
                self.mission = 2

        if self.mission == 2:
            ## Searching Path
            if self.WP_index == 0:
                self.PreWayPoint[0] = self.Cur_Pos_m[0]
                self.PreWayPoint[1] = self.Cur_Pos_m[1]
                self.PreWayPoint[2] = self.Cur_Pos_m[2]
            else:
                self.PreWayPoint[0] = self.WayPoint_X[self.WP_index-1]
                self.PreWayPoint[1] = self.WayPoint_Y[self.WP_index-1]
                self.PreWayPoint[2] = self.WayPoint_Z[self.WP_index-1]

            self.delPos[0] = self.WayPoint_X[self.WP_index] - self.Cur_Pos_m[0]
            self.delPos[1] = self.WayPoint_Y[self.WP_index] - self.Cur_Pos_m[1]
            self.delPos[2] = self.WayPoint_Z[self.WP_index] - self.Cur_Pos_m[2]

            self.WP_dist = np.sqrt(self.delPos[0]*self.delPos[0] + self.delPos[1]*self.delPos[1])
            self.WP_dir = np.arctan2(self.WayPoint_Y[self.WP_index]-self.PreWayPoint[1]+eps, self.WayPoint_X[self.WP_index]-self.PreWayPoint[0])

            self.GoalAction_target.data.append(3)    # Mission
            self.GoalAction_target.data.append(self.WayPoint_X[self.WP_index])  # init x
            self.GoalAction_target.data.append(self.WayPoint_Y[self.WP_index])  # init y
            self.GoalAction_target.data.append(self.WayPoint_Z[self.WP_index])  # init z
            self.GoalAction_target.data.append(self.Init_heading) # init r
            #self.GoalAction_target.data.append(self.WP_dir) # init r
            self.GoalAction_target.data.append(7.0)  # init vx
            self.GoalAction_target.data.append(1.0)  # init vz
            self.pub_GoalAction_target.publish(self.GoalAction_target)
            self.GoalAction_target.data = []

            if self.WP_dist < self.WP_eta:
                self.WP_index = self.WP_index + 1
                if self.WP_index == self.WP_num:
                    self.WP_index = 0

            #print(self.count_balloon, self.mission)

        if self.mission == 3:
            ## Tracking Mode
            self.GoalAction_target.data.append(6)    # Mission
            self.GoalAction_target.data.append(0.0) # init x
            self.GoalAction_target.data.append(0.0) # init y
            self.GoalAction_target.data.append(2.0)  # init z
            self.GoalAction_target.data.append(self.Init_heading) # init r
            self.GoalAction_target.data.append(3.0)  # init vx
            self.GoalAction_target.data.append(1.0)  # init vz
            self.pub_GoalAction_target.publish(self.GoalAction_target)
            self.GoalAction_target.data = []

        if self.mission == 4:
            ## Landing Waypoint

            if self.flag_landing == 0:
                self.PreWayPoint[0] = self.Cur_Pos_m[0]
                self.PreWayPoint[1] = self.Cur_Pos_m[1]
                self.PreWayPoint[2] = self.Cur_Pos_m[2]
                self.flag_landing = 1

            self.delPos[0] = self.LandPoint[0] - self.Cur_Pos_m[0]
            self.delPos[1] = self.LandPoint[1] - self.Cur_Pos_m[1]
            self.delPos[2] = self.LandPoint[2] - self.Cur_Pos_m[2]

            self.WP_dist = np.sqrt(self.delPos[0]*self.delPos[0] + self.delPos[1]*self.delPos[1])
            self.WP_dir = np.arctan2(self.LandPoint[1]-self.PreWayPoint[1], self.LandPoint[0]-self.PreWayPoint[0]+eps)

            self.GoalAction_target.data.append(3)    # Mission
            self.GoalAction_target.data.append(self.LandPoint[0]) # init x
            self.GoalAction_target.data.append(self.LandPoint[1]) # init y
            self.GoalAction_target.data.append(self.LandPoint[2])  # init z
            self.GoalAction_target.data.append(self.WP_dir) # init r
            self.GoalAction_target.data.append(3.0)  # init vx
            self.GoalAction_target.data.append(1.0)  # init vz
            self.pub_GoalAction_target.publish(self.GoalAction_target)
            self.GoalAction_target.data = []

            if self.WP_dist < 1.0:
                self.mission = 5

        if self.mission == 5:
            ## Landing
            self.GoalAction_target.data.append(2)    # Mission
            self.GoalAction_target.data.append(self.LandPoint[0]) # init x
            self.GoalAction_target.data.append(self.LandPoint[1]) # init y
            self.GoalAction_target.data.append(self.LandPoint[2])  # init z
            self.GoalAction_target.data.append(self.WP_dir) # init r
            self.GoalAction_target.data.append(3.0)  # init vx
            self.GoalAction_target.data.append(-0.5)  # init vz
            self.pub_GoalAction_target.publish(self.GoalAction_target)
            self.GoalAction_target.data = []

def main():
    ros = ros_class()
    rospy.init_node('mission_target', anonymous=True)

    #rospy.Subscriber("/target/odom", Odometry, ros.target_odom_callback, queue_size=2)
    rospy.Subscriber("/target/mavros/global_position/local", Odometry, ros.target_odom_callback, queue_size=2)
    rospy.Subscriber("/target/mission", UInt8, ros.target_mission_callback, queue_size=1)
    ros.pub_GoalAction_target = rospy.Publisher("/target/GoalAction", Float32MultiArray, queue_size=10)

    rate = rospy.Rate(20)  # 20hz

    ros.reset()

    while not rospy.is_shutdown():

        if (ros.t_cur % 1.0) == 0:
            print("[time] %.3f [mission] %d [WP index] %d" % (ros.t_cur, ros.mission, ros.WP_index))
            #print("[flag] %d [cur]  %.3f  %.3f  %.3f" % (ros.flag, ros.Cur_Pos_m[0], ros.Cur_Pos_m[1], ros.Cur_Pos_m[2]))
            print("\n\n\n")

        if ros.flag_mission == 1:
            ros.mission_flow()

        #
        # Init --> Disarm and Ready to Arming
        #
        if ros.flag_target == 0:
            ros.GoalAction_target.data.append(0)  # Mission
            ros.pub_GoalAction_target.publish(ros.GoalAction_target)
            ros.GoalAction_target.data = []

        #
        # takeoff
        #
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
        if ros.flag_target == 3:
            ros.GoalAction_target.data.append(2)  # Mission
            ros.GoalAction_target.data.append(20.0)  # init x
            ros.GoalAction_target.data.append(50.0)  # init y
            ros.GoalAction_target.data.append(2.0)  # init z
            ros.GoalAction_target.data.append(0.0)  # init r
            ros.GoalAction_target.data.append(1.0)  # init vx
            ros.GoalAction_target.data.append(-0.5)  # init vz
            ros.pub_GoalAction_target.publish(ros.GoalAction_target)
            ros.GoalAction_target.data = []

        #
        # mission
        #
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

        ros.count = ros.count + 1
        ros.t_cur = ros.count / 20.0
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
