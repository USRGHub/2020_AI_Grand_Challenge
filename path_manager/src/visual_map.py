#!/usr/bin/env python
# license removed for brevity

import numpy as np
import cv2
import sys
import paramiko
import string
import os
import rospy
import tf

import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String, Float32MultiArray, Header, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray


class visual_plot:
    def __init__(self):
        self.robot_namespace = 'c'
        self.pose = [0.0, 0.0, 0.0]
        self.q = []
        self.euler = [0.0, 0.0, 0.0]
        self.D2R = 3.14159265/180.0

        self.path_cur = Path()
        self.map_marker = MarkerArray()

        self.pub_path = 1
        self.pub_pos = 1
        self.pub_map = 1

        self.transparents = 1.0

        self.poseavailable = 0

    def rviz_map(self):
        # ground plane
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 150.0
        box_marker.scale.y = 150.0
        box_marker.scale.z = 0.1
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(30.0, 50.0, 0.1)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_plane"
        self.map_marker.markers.append(box_marker)

        #
        ####################### basebox #########################
        #

        # basebox 1
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 6.0
        box_marker.scale.y = 6.0
        box_marker.scale.z = 8.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 0.0, 4.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_basebox_1"
        self.map_marker.markers.append(box_marker)

        # basebox 2
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 6.0
        box_marker.scale.y = 6.0
        box_marker.scale.z = 8.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 0.0, 4.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_basebox_2"
        self.map_marker.markers.append(box_marker)

        # basebox 3
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 6.0
        box_marker.scale.y = 6.0
        box_marker.scale.z = 8.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 100.0, 4.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_basebox_3"
        self.map_marker.markers.append(box_marker)

        # basebox 4
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 6.0
        box_marker.scale.y = 6.0
        box_marker.scale.z = 8.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 100.0, 4.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_basebox_4"
        self.map_marker.markers.append(box_marker)

        # basebox 5
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 6.0
        box_marker.scale.y = 6.0
        box_marker.scale.z = 8.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 50.0, 4.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_basebox_5"
        self.map_marker.markers.append(box_marker)

        # basebox 6
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 6.0
        box_marker.scale.y = 6.0
        box_marker.scale.z = 8.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 50.0, 4.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_basebox_6"
        self.map_marker.markers.append(box_marker)

        #
        ####################### Column #########################
        #

        # column 1
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 20.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 0.0, 10.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_column_1"
        self.map_marker.markers.append(box_marker)

        # column 2
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 20.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 0.0, 10.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_column_2"
        self.map_marker.markers.append(box_marker)

        # column 3
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 20.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 100.0, 10.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_column_3"
        self.map_marker.markers.append(box_marker)

        # column 4
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 20.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 100.0, 10.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_column_4"
        self.map_marker.markers.append(box_marker)

        # column 5
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 20.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 50.0, 10.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_column_5"
        self.map_marker.markers.append(box_marker)

        # column 6
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 20.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 50.0, 10.0)
        box_att = Quaternion(0, 0, 0, 1)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_column_6"
        self.map_marker.markers.append(box_marker)


        #
        ####################### Roof #########################
        #

        # roof 1
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 100.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 50.0, 20.0)
        box_att = Quaternion(0.7071, 0, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_1"
        self.map_marker.markers.append(box_marker)

        # roof 2
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 100.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 50.0, 20.0)
        box_att = Quaternion(0.7071, 0, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_2"
        self.map_marker.markers.append(box_marker)

        # roof 3
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 40.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(20.0, 100.0, 20.0)
        box_att = Quaternion(0, 0.7071, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_3"
        self.map_marker.markers.append(box_marker)

        # roof 4
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 40.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(20.0, 0.0, 20.0)
        box_att = Quaternion(0, 0.7071, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_4"
        self.map_marker.markers.append(box_marker)

        # roof 5
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 100.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(0.0, 50.0, 0.1)
        box_att = Quaternion(0.7071, 0, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_5"
        self.map_marker.markers.append(box_marker)

        # roof 6
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 100.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(40.0, 50.0, 0.1)
        box_att = Quaternion(0.7071, 0, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_6"
        self.map_marker.markers.append(box_marker)

        # roof 7
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 40.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(20.0, 100.0, 0.1)
        box_att = Quaternion(0, 0.7071, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_7"
        self.map_marker.markers.append(box_marker)

        # roof 8
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.header = Header(frame_id='map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 40.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(20.0, 0.0, 0.1)
        box_att = Quaternion(0, 0.7071, 0, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "mbzirc_roof_8"
        self.map_marker.markers.append(box_marker)


        # tunnel 1
        '''
        box_marker = Marker()
        box_marker.type = Marker.MESH_RESOURCE
        box_marker.mesh_resource = "package://plot_data/mesh/AIGC_PIPE.dae"
        box_marker.header = Header(frame_id='scout/map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(25.75, -5.0, 1.55)
        box_att = Quaternion(0, 0, 0.7071, 0.7071)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "tunnel_1"
        self.map_marker.markers.append(box_marker)
        '''

        '''
        # pole 1
        box_marker = Marker()
        box_marker.type = Marker.CYLINDER
        box_marker.header = Header(frame_id='scout/map')
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 2.0
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = self.transparents
        box_pos = Point(14.4, -1.18, 1.0)
        box_att = Quaternion(0, 0, 0.0, 1.0)
        box_marker.pose = Pose(box_pos, box_att)
        box_marker.ns = "pole_1"
        self.map_marker.markers.append(box_marker)
        '''


def main():

    visual = visual_plot()
    visual.robot_namespace = rospy.get_namespace()
    rospy.init_node('visual_map', anonymous=True)

    visual.pub_map = rospy.Publisher("map_rviz", MarkerArray, queue_size=1)

    visual.rviz_map()

    rate = rospy.Rate(1)  # 20hz
    while not rospy.is_shutdown():

        visual.pub_map.publish(visual.map_marker)

        rate.sleep()

    visual.path_cur = []


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
