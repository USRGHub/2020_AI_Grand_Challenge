#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define R2D      180.0/3.141592

using namespace std;
using namespace cv;
using namespace ros;

void toEulerAngle(Vec4f &);

Vec3f _state;
Vec4f _quaternion;
mavros_msgs::Altitude altitude;
sensor_msgs::LaserScan terarange;
float  Cur_Att_rad[3];
double q[4];
float height_m = 0.0;

static void QuaterniontoEuler(float& roll, float& pitch, float& yaw);

void callback_mav_altitude(const mavros_msgs::Altitude::ConstPtr& msg)
{
    altitude = *msg;
}

void callback_terarange(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    terarange = *msg;

    height_m = terarange.ranges[0];
    //printf("%.3f\n",terarange.ranges[0]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle nh_;

  //Subscriber sub_alt = nh_.subscribe<mavros_msgs::Altitude> ("/mavros/altitude", 1, callback_mav_altitude);
  Subscriber sub_tera = nh_.subscribe<sensor_msgs::LaserScan> ("/teraranger", 1, callback_terarange);

  ros::Publisher uav_vel = nh_.advertise<nav_msgs::Odometry> ("/nav_position", 10);
  ros::Publisher uav_pose = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  tf::TransformListener listener;

  ros::Rate rate(40.0);  

  while( ok() )
  {
    tf::StampedTransform transform;
    nav_msgs::Odometry nav_msg;
    geometry_msgs::PoseStamped pos_msg;

    try{
      //listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      listener.lookupTransform("/world", "/uav/imu", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }

    _state = Vec3f(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    _quaternion = Vec4f(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());


    nav_msg.pose.pose.position.x = _state[0];
    nav_msg.pose.pose.position.y = _state[1];
    //nav_msg.pose.pose.position.z = altitude.bottom_clearance;
    nav_msg.pose.pose.position.z = _state[2];
    nav_msg.pose.pose.orientation.x = _quaternion[0];
    nav_msg.pose.pose.orientation.y = _quaternion[1];
    nav_msg.pose.pose.orientation.z = _quaternion[2];
    nav_msg.pose.pose.orientation.w = _quaternion[3];
    uav_vel.publish(nav_msg);

    pos_msg.header.stamp = ros::Time::now();
    pos_msg.header.frame_id = "map";
    pos_msg.pose.position.x = _state[0];
    pos_msg.pose.position.y = _state[1];
    pos_msg.pose.position.z = height_m;
    pos_msg.pose.orientation.x = _quaternion[0];
    pos_msg.pose.orientation.y = _quaternion[1];
    pos_msg.pose.orientation.z = _quaternion[2];
    pos_msg.pose.orientation.w = _quaternion[3];
    uav_pose.publish(pos_msg);

    q[0] = _quaternion[0];
    q[1] = _quaternion[1];
    q[2] = _quaternion[2];
    q[3] = _quaternion[3];

    QuaterniontoEuler(Cur_Att_rad[0],Cur_Att_rad[1],Cur_Att_rad[2]);

    //ROS_INFO("PosX: %.3f, PosY: %.3f, PosZ: %.3f Heading : %.3f\n", _state[0], _state[1],  altitude.bottom_clearance, Cur_Att_rad[2]*R2D);
    ROS_INFO("PosX: %.3f, PosY: %.3f, PosZ: %.3f Heading : %.3f\n", _state[0], _state[1],  height_m, Cur_Att_rad[2]*R2D);
    //ROS_INFO("X: %.3f, Y: %.3f, Z: %.3f, w: %.3f\n", _quaternion[0], _quaternion[1], _quaternion[2], _quaternion[3]);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

static void QuaterniontoEuler(float& roll, float& pitch, float& yaw)
{

    // roll (x-axis rotation)
    float t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    float t1 = +1.0 - 2.0 * (q[0] * q[0] + q[1]*q[1]);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = +2.0 * (q[3] * q[1] - q[2] * q[0]);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = -std::asin(t2);

    // yaw (z-axis rotation)
    float t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    float t4 = +1.0 - 2.0 * (q[1]*q[1] + q[2] * q[2]);
    yaw = std::atan2(t3, t4);
}

