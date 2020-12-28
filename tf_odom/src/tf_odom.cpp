#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace ros;

static void QuaterniontoEuler(double* quat, float& roll, float& pitch, float& yaw);

float imu_Att_rad[3];

nav_msgs::Odometry nav_msg;
nav_msgs::Odometry odom_msg;

ros::Publisher odom_frame;

void Callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_msg.header = msg->header;
    //odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "scout/map";
    odom_msg.child_frame_id = "scout/odom";
    odom_msg.pose = msg->pose;
    odom_msg.twist = msg->twist;
    odom_frame.publish(odom_msg);

    tf::Quaternion q;
    q[0] = msg->pose.pose.orientation.x;
    q[1] = msg->pose.pose.orientation.y;
    q[2] = msg->pose.pose.orientation.z;
    q[3] = msg->pose.pose.orientation.w;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "scout/map", "scout/odom_pose"));
}

int main(int agrc, char** agrv)
{
    ros::init(agrc, agrv, "tf_odom");
    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_pub;

    //ros::Subscriber sub_pose = nh_sub.subscribe("scout/qsf/pose", 1, &Callback_pose);
    ros::Subscriber sub_odom = nh_sub.subscribe("odom_gazebo", 1, &Callback_odom);
    odom_frame = nh_pub.advertise<nav_msgs::Odometry>("odom", 1);

    ros::Rate rate(100.0);

    while( ok() )
    {

        tf::Quaternion q;
        q[0] = 0.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 1.0;

        static tf::TransformBroadcaster odom_br;
        tf::Transform odom_tf;
        odom_tf.setOrigin(tf::Vector3(-0.15, 0.0, 0.0));
        odom_tf.setRotation(q);
        odom_br.sendTransform(tf::StampedTransform(odom_tf, ros::Time::now(), "scout/odom", "scout/base_link"));

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

