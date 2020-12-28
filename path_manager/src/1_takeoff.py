#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray, Float32MultiArray


def main():
    rospy.init_node('msg_generator', anonymous=True)
    pub_devinfo_uav1 = rospy.Publisher("/uav1/GoalAction", Float32MultiArray, queue_size=2)
    pub_devinfo_uav2 = rospy.Publisher("/uav2/GoalAction", Float32MultiArray, queue_size=2)
    rate = rospy.Rate(1)

    GoalAction_uav1 = Float32MultiArray()
    del GoalAction_uav1.data[:]  # clear msg
    GoalAction_uav1.data.append(1)  # Mission
    GoalAction_uav1.data.append(0.0)  # init x
    GoalAction_uav1.data.append(0.0)  # init y
    GoalAction_uav1.data.append(4.0)  # init z
    GoalAction_uav1.data.append(0.0)  # init r
    GoalAction_uav1.data.append(1.0)  # init vx
    GoalAction_uav1.data.append(1.5)  # init vz

    GoalAction_uav2 = Float32MultiArray()
    del GoalAction_uav2.data[:]  # clear msg
    GoalAction_uav2.data.append(1)  # Mission
    GoalAction_uav2.data.append(8.0)  # init x
    GoalAction_uav2.data.append(2.0)  # init y
    GoalAction_uav2.data.append(2.0)  # init z
    GoalAction_uav2.data.append(0.0)  # init r
    GoalAction_uav2.data.append(1.0)  # init vx
    GoalAction_uav2.data.append(1.5)  # init vz


    while not rospy.is_shutdown():
        print("publish")

        pub_devinfo_uav1.publish(GoalAction_uav1)
        pub_devinfo_uav2.publish(GoalAction_uav2)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
