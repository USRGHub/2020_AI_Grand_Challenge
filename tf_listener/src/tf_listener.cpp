#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/Altitude.h>
#include <std_msgs/Float32MultiArray.h>
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
float  mav_Pos_m[3];
float  mav_Att_rad[3];
float  imu_Att_rad[3];

double q[4];
float height_m = 0.1;
float height_RL = 0.1;
float height_LPF = 0.1;
float height_pre = 0.1;
float captHeight = 0.1;
float PI = 3.1415926535;

float Hz = 60.0;
float LIMIT = 0.5;
float mission = 0.0;

string robot_namespace;

static void QuaterniontoEuler(double* quat, float& roll, float& pitch, float& yaw);

void callback_terarange(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    terarange = *msg;

    if (isinf(terarange.ranges[0]))
    {
        height_m = 0.1;
    }
    else
    {
        height_m = terarange.ranges[0];
    }

    if (isnan(terarange.ranges[0]))
        height_m = height_pre;

    height_pre = height_m;
    //printf("%.3f\n",terarange.ranges[0]);
}

void missionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    mission = msg->data[0];
}

float LPF(float data, float data_pre, float freq)
{
    float res;
    float K_LPF;
    float delT = 1/Hz;

    K_LPF = freq*2*PI*delT / (1 + freq*2*PI*delT);

    res = (data - data_pre) * K_LPF + data_pre;

    return res;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle nh_;
  Subscriber sub_tera = nh_.subscribe<sensor_msgs::LaserScan> ("/scout/teraranger", 1, callback_terarange);
  ros::Publisher uav_pose = nh_.advertise<geometry_msgs::PoseStamped>("/scout/mavros/vision_pose/pose", 10);
  tf::TransformListener listener;

  ros::Rate rate(60.0);

  while( ok() )
  {
    tf::StampedTransform transform;
    nav_msgs::Odometry nav_msg;
    geometry_msgs::PoseStamped pos_msg;

    try{
      listener.lookupTransform("/scout/map", "/scout/horizontal_laser_link", ros::Time(0), transform);      
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }

    _state = Vec3f(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    _quaternion = Vec4f(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());


    float rate_height = (height_m - height_RL)*Hz;

    if (rate_height > LIMIT)
        rate_height = LIMIT;
    if (rate_height < -LIMIT)
        rate_height = -LIMIT;

    height_RL = height_RL + rate_height/Hz;

    height_LPF = LPF(height_RL, height_pre, 2.0);
    height_pre = height_LPF;

    pos_msg.header.stamp = ros::Time::now();
    pos_msg.header.frame_id = "scout/map";
    pos_msg.pose.position.x = _state[0];
    pos_msg.pose.position.y = _state[1];

    if (mission == 7.0)
    {
        pos_msg.pose.position.z = height_LPF;
    }
    else
    {
        pos_msg.pose.position.z = height_m;
    }
    //pos_msg.pose.position.z = height_m;
    pos_msg.pose.orientation.x = _quaternion[0];
    pos_msg.pose.orientation.y = _quaternion[1];
    pos_msg.pose.orientation.z = _quaternion[2];
    pos_msg.pose.orientation.w = _quaternion[3];
    uav_pose.publish(pos_msg);

    q[0] = _quaternion[0];
    q[1] = _quaternion[1];
    q[2] = _quaternion[2];
    q[3] = _quaternion[3];

    QuaterniontoEuler(q, Cur_Att_rad[0],Cur_Att_rad[1],Cur_Att_rad[2]);

    //ROS_INFO("PosX: %.3f, PosY: %.3f, PosZ: %.3f Heading : %.3f\n", _state[0], _state[1],  altitude.bottom_clearance, Cur_Att_rad[2]*R2D);
    ROS_INFO("[vision] PosX: %.3f, PosY: %.3f, PosZ: %.3f Att : %.3f, %.3f %.3f\n", _state[0], _state[1],  height_m, Cur_Att_rad[0]*R2D, Cur_Att_rad[1]*R2D, Cur_Att_rad[2]*R2D);
    //ROS_INFO("[local]  PosX: %.3f, PosY: %.3f, PosZ: %.3f Att : %.3f, %.3f %.3f\n", mav_Pos_m[0], mav_Pos_m[1],  mav_Pos_m[2], mav_Att_rad[0]*R2D, mav_Att_rad[1]*R2D, mav_Att_rad[2]*R2D);
    //ROS_INFO("[imu]                                       Att : %.3f, %.3f %.3f\n", imu_Att_rad[0]*R2D, imu_Att_rad[1]*R2D, imu_Att_rad[2]*R2D);
    //ROS_INFO("X: %.3f, Y: %.3f, Z: %.3f, w: %.3f\n", _quaternion[0], _quaternion[1], _quaternion[2], _quaternion[3]);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

static void QuaterniontoEuler(double* quat, float& roll, float& pitch, float& yaw)
{

    // roll (x-axis rotation)
    float t0 = +2.0 * (quat[3] * quat[0] + quat[1] * quat[2]);
    float t1 = +1.0 - 2.0 * (quat[0] * quat[0] + quat[1]*quat[1]);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = +2.0 * (quat[3] * quat[1] - quat[2] * quat[0]);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = -std::asin(t2);

    // yaw (z-axis rotation)
    float t3 = +2.0 * (quat[3] * quat[2] + quat[0] * quat[1]);
    float t4 = +1.0 - 2.0 * (quat[1]*quat[1] + quat[2] * quat[2]);
    yaw = std::atan2(t3, t4);
}

