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

        self.pub_GoalAction_uav2 = 0
        self.pub_Servo_1 = 0
        self.pub_Servo_2 = 0
        self.pub_Servo_3 = 0

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
        self.GoalAction_uav2 = Float32MultiArray()

        self.flag_uav2 = -1
        self.PubMissionCallback = 0

        self.sonar = 100.0
        self.flag_servo_start = 0
        self.flag_prepare_catch = 0
        self.flag_prepare_release = 0
        self.time_mission = 0

        ## Detection
        self.flag_detection = 0
        self.size = [10, 10]
        self.impos = [480, 360]
        self.tar_dist = 10.0

        ## Mission Planning
        self.flag_mission = 0
        self.flag_landing = 0
        self.flag_time = 0
        self.mission = 0

        self.WP_index = 0
        self.WP_num = 2
        self.WP_dist = 10.0
        self.WP_dir = 0.0
        self.WP_eta = 0.5
        #self.WayPoint_X = [0.0, 21.907, -36.049, -33.461, 43.813, -38.637]
        #self.WayPoint_Y = [0.0, 4.483, 20.012, 29.671, 8.966, 10.353]

        # (25.000, 50.000) -> (23.201, 9.313)
        # (24.000, 50.000) -> (22.942, 8.347)
        # (23.000, 50.000) -> (22.683, 7.381)
        # (22.000, 50.000) -> (22.424, 6.415)
        # (21.000, 50.000) -> (22.166, 5.449)
        # (20.000, 50.000) -> (21.907, 4.483)
        # (19.000, 50.000) -> (21.648, 3.517)
        # (18.000, 50.000) -> (21.389, 2.551)
        # (17.000, 50.000) -> (21.130, 1.585)

        self.WayPoint_X = [0.0, 21.907, -36.049, -33.461, 43.813, -38.637]
        self.WayPoint_Y = [0.0, 4.483, 20.012, 29.671, 8.966, 10.353]
        self.WayPoint_Z = [ 3.0,  5.0,  2.0,  2.0,  2.0,  2.0]
        self.delPos = [0.0, 0.0, 0.0]
        self.PreWayPoint = [0.0, 0.0, 0.0]
        self.LandPoint = [17.387, -4.659, 2.0]
        self.Init_heading = 1.309
        self.Start_Pos = [10, 75]
        self.Start_Pos_x = -4.830
        self.Start_Pos_y = 1.294
        self.GeoFenceBound = 0.0
        self.BinPoint = [38.637, -10.353, 2.0]

        self.count_balloon = 0
        self.count_balloon_not = 0
        self.count_disappear = 0


    def reset(self):
        #angle = np.arctan2(random()-0.5, random()-0.5)
        #self.init[0] = 1.0*np.cos(angle)
        #self.init[1] = 1.0*np.sin(angle)
        #self.init[2] = (random() + 1.0)

        # Case 1.
        self.init[0] = 0.0
        self.init[1] = 0.0
        self.init[2] = 2.0

        self.mission = 3

        self.t_cur = 0.0
        self.count = 0

    def uav2_odom_callback(self, data):

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

    def detection_callback(self, data):
        self.flag_detection = data.data[0]
        self.size[0] = data.data[1]
        self.size[1] = data.data[2]
        self.impos[0] = data.data[3]
        self.impos[1] = data.data[4]

    def uav2_mission_callback(self, data):
        self.flag_uav2 = data.data

    def sonar_callback(self, data):
        self.sonar = data.data

    def mission_flow(self):
        if self.mission == 1:
            ## Mission Start & Take-off
            self.GoalAction_uav2.data.append(1)    # Mission
            self.GoalAction_uav2.data.append(0.0) # init x
            self.GoalAction_uav2.data.append(0.0) # init y
            self.GoalAction_uav2.data.append(5.0)  # init z
            self.GoalAction_uav2.data.append(self.Init_heading) # init r
            self.GoalAction_uav2.data.append(1.0)  # init vx
            self.GoalAction_uav2.data.append(1.5)  # init vz
            self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
            self.GoalAction_uav2.data = []
            dist = np.fabs(5.0 - self.Cur_Pos_m[2])
            if dist < 0.2:
                self.mission = 2
            if self.Cur_Pos_m[2] > 2.0:
                self.mission = 2

        if self.mission == 2:

            self.GoalAction_uav2.data.append(3)    # Mission
            self.GoalAction_uav2.data.append(self.WayPoint_X[1])  # init x
            self.GoalAction_uav2.data.append(self.WayPoint_Y[1])  # init y
            self.GoalAction_uav2.data.append(self.WayPoint_Z[1])  # init z
            self.GoalAction_uav2.data.append(self.Init_heading + 1.57079 - 0.32) # init r
            self.GoalAction_uav2.data.append(2.0)  # init vx
            self.GoalAction_uav2.data.append(1.0)  # init vz
            self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
            self.GoalAction_uav2.data = []

            #self.WayPoint_X[0] = self.WayPoint_X[1]
            #self.WayPoint_Y[0] = self.WayPoint_Y[1]
            #self.WayPoint_Z[0] = self.WayPoint_Z[1]

            if self.flag_detection == 1:
                self.count_balloon = self.count_balloon + 1

            if self.count_balloon > 20.0*0.5:
                self.mission = 3
                self.count_balloon = 0

            #print(self.count_balloon, self.mission)

        if self.mission == 3:

            diag = np.sqrt(self.size[0]*self.size[0] + self.size[1]*self.size[1])
            self.tar_dist = 470.18*pow(2.5*diag, -1.046)

            if self.tar_dist < 8.0:
                ## Tracking Mode
                self.GoalAction_uav2.data.append(6)    # Mission
                self.GoalAction_uav2.data.append(0.0) # init x
                self.GoalAction_uav2.data.append(0.0) # init y
                self.GoalAction_uav2.data.append(2.0)  # init z
                self.GoalAction_uav2.data.append(self.Init_heading + 1.57079 - 0.32) # init r
                self.GoalAction_uav2.data.append(-0.8)  # init vx
                self.GoalAction_uav2.data.append(1.0)  # init vz
                self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
                self.GoalAction_uav2.data = []
            else:
                ## WayPoint Mode
                self.GoalAction_uav2.data.append(3)  # Mission
                self.GoalAction_uav2.data.append(self.WayPoint_X[1])  # init x
                self.GoalAction_uav2.data.append(self.WayPoint_Y[1])  # init y
                self.GoalAction_uav2.data.append(self.WayPoint_Z[1])  # init z
                self.GoalAction_uav2.data.append(self.Init_heading + 1.57079 - 0.32) # init r
                self.GoalAction_uav2.data.append(2.0)  # init vx
                self.GoalAction_uav2.data.append(1.0)  # init vz
                self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
                self.GoalAction_uav2.data = []

            if self.flag_detection == 1:
                self.count_balloon_not = 0

                #self.WayPoint_Y[1] = self.Cur_Pos_m[1]
                self.WayPoint_Z[1] = self.Cur_Pos_m[2]

                #del_y = self.WayPoint_Y[0] - self.Cur_Pos_m[1]
                #del_z = self.WayPoint_Z[0] - self.Cur_Pos_m[2]
                #dist = np.sqrt(del_y*del_y + del_z*del_z)

                #if dist < 1.0:
                #    self.WayPoint_X[1] = self.Cur_Pos_m[0]
                #    self.WayPoint_Y[1] = self.Cur_Pos_m[1]
                #    self.WayPoint_Z[1] = self.Cur_Pos_m[2]
                #else:
                #    self.mission = 2

            else:
                self.count_balloon_not = self.count_balloon_not + 1

            if self.count_balloon_not > 20.0*0.5:
                self.mission = 2

            #if self.size[0]*self.size[1] > 9000.0:
            #       self.mission = 4

            '''
            if self.size[0]*self.size[1] > 8000.0:
                if self.flag_detection == 0:
                    self.count_disappear = self.count_disappear + 1

                if self.count_disappear > 20.0 * 2.0:
                    self.mission = 4
                    self.count_disappear = 0
            '''

        if self.mission == 4:
            ## Bin Waypoint

            if self.flag_landing == 0:
                self.PreWayPoint[0] = self.Cur_Pos_m[0]
                self.PreWayPoint[1] = self.Cur_Pos_m[1]
                self.PreWayPoint[2] = self.Cur_Pos_m[2]
                self.flag_landing = 1

            self.delPos[0] = self.BinPoint[0] - self.Cur_Pos_m[0]
            self.delPos[1] = self.BinPoint[1] - self.Cur_Pos_m[1]
            self.delPos[2] = self.BinPoint[2] - self.Cur_Pos_m[2]

            self.WP_dist = np.sqrt(self.delPos[0]*self.delPos[0] + self.delPos[1]*self.delPos[1])
            self.WP_dir = np.arctan2(self.LandPoint[1]-self.PreWayPoint[1], self.LandPoint[0]-self.PreWayPoint[0]+eps)

            self.GoalAction_uav2.data.append(3)    # Mission
            self.GoalAction_uav2.data.append(self.BinPoint[0]) # init x
            self.GoalAction_uav2.data.append(self.BinPoint[1]) # init y
            self.GoalAction_uav2.data.append(self.BinPoint[2])  # init z
            self.GoalAction_uav2.data.append(self.WP_dir) # init r
            self.GoalAction_uav2.data.append(3.0)  # init vx
            self.GoalAction_uav2.data.append(1.0)  # init vz
            self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
            self.GoalAction_uav2.data = []

            if self.WP_dist < 1.0:
                if self.flag_prepare_release == 0:
                    self.time_mission = self.t_cur
                    self.flag_prepare_release = 1
                
                self.GoalAction_uav2.data.append(3)    # Mission
                self.GoalAction_uav2.data.append(self.BinPoint[0]) # init x
                self.GoalAction_uav2.data.append(self.BinPoint[1]) # init y
                self.GoalAction_uav2.data.append(self.BinPoint[2])  # init z
                self.GoalAction_uav2.data.append(self.Init_heading+1.57079) # init r
                self.GoalAction_uav2.data.append(1.0)  # init vx
                self.GoalAction_uav2.data.append(1.0)  # init vz
                self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
                self.GoalAction_uav2.data = []
                
                if self.t_cur-self.time_mission > 10.0:
                    self.mission = 5

        if self.mission == 5:
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

            self.GoalAction_uav2.data.append(3)    # Mission
            self.GoalAction_uav2.data.append(self.LandPoint[0]) # init x
            self.GoalAction_uav2.data.append(self.LandPoint[1]) # init y
            self.GoalAction_uav2.data.append(self.LandPoint[2])  # init z
            self.GoalAction_uav2.data.append(self.WP_dir) # init r
            self.GoalAction_uav2.data.append(3.0)  # init vx
            self.GoalAction_uav2.data.append(1.0)  # init vz
            self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
            self.GoalAction_uav2.data = []

            if self.WP_dist < 1.0:
                self.mission = 6

        if self.mission == 6:
            ## Landing
            self.GoalAction_uav2.data.append(2)    # Mission
            self.GoalAction_uav2.data.append(self.LandPoint[0]) # init x
            self.GoalAction_uav2.data.append(self.LandPoint[1]) # init y
            self.GoalAction_uav2.data.append(self.LandPoint[2])  # init z
            self.GoalAction_uav2.data.append(self.WP_dir) # init r
            self.GoalAction_uav2.data.append(3.0)  # init vx
            self.GoalAction_uav2.data.append(-0.5)  # init vz
            self.pub_GoalAction_uav2.publish(self.GoalAction_uav2)
            self.GoalAction_uav2.data = []

def main():
    ros = ros_class()
    rospy.init_node('mission_uav2', anonymous=True)

    #rospy.Subscriber("/uav2/odom", Odometry, ros.uav2_odom_callback, queue_size=2)
    rospy.Subscriber("/uav2/mavros/global_position/local", Odometry, ros.uav2_odom_callback, queue_size=2)
    rospy.Subscriber("/uav2/detection", Float32MultiArray, ros.detection_callback, queue_size=2)
    rospy.Subscriber("/uav2/arduino/ultrasound1", Float32, ros.sonar_callback, queue_size=2)

    ros.pub_Servo_1 = rospy.Publisher("/uav2/arduino/servo1", UInt16, queue_size=1)
    ros.pub_Servo_2 = rospy.Publisher("/uav2/arduino/servo2", UInt16, queue_size=1)
    ros.pub_Servo_3 = rospy.Publisher("/uav2/arduino/servo3", UInt16, queue_size=1)

    rospy.Subscriber("/uav2/mission", UInt8, ros.uav2_mission_callback, queue_size=1)
    ros.pub_GoalAction_uav2 = rospy.Publisher("/uav2/GoalAction", Float32MultiArray, queue_size=10)

    rate = rospy.Rate(20)  # 20hz

    ros.reset()

    while not rospy.is_shutdown():

        if (ros.t_cur*10.0 % 1.0) == 0:
            print("[time] %.3f [mission] %d [tar dist] %.3f" % (ros.t_cur, ros.mission, ros.tar_dist))
            #print("[flag] %d [cur]  %.3f  %.3f  %.3f" % (ros.flag, ros.Cur_Pos_m[0], ros.Cur_Pos_m[1], ros.Cur_Pos_m[2]))
            print("\n\n\n")

        if ros.flag_mission == 1:
            ros.mission_flow()

        if ros.t_cur > 5.0:
            ros.flag_servo_start = 1

        servo_1 = UInt16()
        servo_2 = UInt16()
        servo_3 = UInt16()

        if ros.flag_servo_start == 1:
            if ros.sonar < 13.0:
                ros.flag_prepare_catch = 1

            if ros.flag_prepare_catch == 1:
                servo_1.data = 120
                servo_2.data = 120
            else:
                servo_1.data = 60
                servo_2.data = 60
        else:
            servo_1.data = 60
            servo_2.data = 60


        if ros.t_cur > 30.0:
            servo_3.data = 70  # open
        else:
            servo_3.data = 150  # close


        ros.pub_Servo_1.publish(servo_1)
        servo_1.data =[]

        ros.pub_Servo_2.publish(servo_2)
        servo_2.data = []

        ros.pub_Servo_3.publish(servo_3)
        servo_3.data = []

        # for debugging
        #if ros.flag_uav2 == 4:
        #    ros.mission_flow()

        #
        # Init --> Disarm and Ready to Arming
        #
        if ros.flag_uav2 == 0:
            ros.GoalAction_uav2.data.append(0)  # Mission
            ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
            ros.GoalAction_uav2.data = []
            ros.PubMissionCallback = 1
            #if ros.t_cur > 2.0:
            #    ros.flag = 1

        #
        # takeoff
        #
        if ros.flag_uav2 == 1:
            ros.GoalAction_uav2.data.append(1)  # Mission
            ros.GoalAction_uav2.data.append(30.0) # init x
            ros.GoalAction_uav2.data.append(50.0) # init y
            ros.GoalAction_uav2.data.append(2.0)  # init z
            ros.GoalAction_uav2.data.append(3.14)  # init r
            ros.GoalAction_uav2.data.append(1.0)  # init vx
            ros.GoalAction_uav2.data.append(1.5)  # init vz
            ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
            ros.GoalAction_uav2.data = []

        #
        # waypoint
        #
        if ros.flag_uav2 == 2:
            ros.GoalAction_uav2.data.append(3)  # Mission
            ros.GoalAction_uav2.data.append(30.0)  # init x
            ros.GoalAction_uav2.data.append(50.0)  # init y
            ros.GoalAction_uav2.data.append(2.0)  # init z
            ros.GoalAction_uav2.data.append(3.14)  # init r
            ros.GoalAction_uav2.data.append(2.0)  # init vx
            ros.GoalAction_uav2.data.append(1.0)  # init vz
            ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
            ros.GoalAction_uav2.data = []

        #
        # landing
        #
        if ros.flag_uav2 == 3:
            ros.GoalAction_uav2.data.append(2)  # Mission
            ros.GoalAction_uav2.data.append(30.0)  # init x
            ros.GoalAction_uav2.data.append(50.0)  # init y
            ros.GoalAction_uav2.data.append(2.0)  # init z
            ros.GoalAction_uav2.data.append(3.14)  # init r
            ros.GoalAction_uav2.data.append(1.0)  # init vx
            ros.GoalAction_uav2.data.append(-0.5)  # init vz
            ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
            ros.GoalAction_uav2.data = []

        #
        # mission
        #
        if ros.flag_uav2 == 4:
            ros.GoalAction_uav2.data.append(6)  # Mission
            ros.GoalAction_uav2.data.append(0.0)  # init x
            ros.GoalAction_uav2.data.append(0.0)  # init y
            ros.GoalAction_uav2.data.append(0.0)  # init z
            ros.GoalAction_uav2.data.append(0.0)  # init r
            ros.GoalAction_uav2.data.append(1.0)  # init vx
            ros.GoalAction_uav2.data.append(1.0)  # init vz
            ros.pub_GoalAction_uav2.publish(ros.GoalAction_uav2)
            ros.GoalAction_uav2.data = []

        ros.count = ros.count + 1
        ros.t_cur = ros.count / 20.0
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
