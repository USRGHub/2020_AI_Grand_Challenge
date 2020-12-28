#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define R2D      180.0/3.141592
#define D2R      3.141592/180.0
#define eps      0.000000001

#define LocalMap_AngNum    60
#define LocalMap_Dist     180
#define LocalMap_AngRange 200.0

using namespace std;
using namespace cv;
using namespace ros;

float Cur_Pos_m[3];
float Cur_Att_rad[3];
int   Cur_Grid[2];
int   Rel_Grid[2];
Vec4f q;
int   flag_ScanInfo = 0;

float ScanAngRange = 70.0;
float ScanAngNum = 64;
float ScanGap = 10;
float ScanBias = 0;

float  bound_x = 0.0;
float  bound_y = 0.0;
float  rangeScan = 0.0;

int bound_Gx = 0;
int bound_Gy = 0;

std::string robot_namespace;
int MapDataAvailable = 0;

ros::Publisher Map_local_pub;
ros::Publisher Mar_tar_pub;
ros::Publisher MarArr_tar_pub;

nav_msgs::OccupancyGrid Map;
std_msgs::Header Cur_Header;

visualization_msgs::Marker Target;
visualization_msgs::MarkerArray TargetArray;
sensor_msgs::LaserScan::ConstPtr scan;

void publish_localmap(void);
static void QuaterniontoEuler(float& roll, float& pitch, float& yaw);

void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Cur_Header = msg->header;
    Cur_Pos_m[0] = msg->pose.position.x;
    Cur_Pos_m[1] = msg->pose.position.y;
    Cur_Pos_m[2] = msg->pose.position.z;

    q[0] = msg->pose.orientation.x;
    q[1] = msg->pose.orientation.y;
    q[2] = msg->pose.orientation.z;
    q[3] = msg->pose.orientation.w;

    QuaterniontoEuler(Cur_Att_rad[0],Cur_Att_rad[1],Cur_Att_rad[2]);

    if(MapDataAvailable == 1)
    {
        Cur_Grid[0] = int((Cur_Pos_m[0] - Map.info.origin.position.x)/Map.info.resolution);
        Cur_Grid[1] = int((Cur_Pos_m[1] - Map.info.origin.position.y)/Map.info.resolution);
    }
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan = msg;
    flag_ScanInfo = 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map");

    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_pos;
    ros::NodeHandle nh_pub;
	
    Subscriber scan_sub = nh_sub.subscribe("/scan", 1, scan_callback);
    Subscriber vis_sub = nh_pos.subscribe("/scout/mavros/local_position/pose", 1, pos_callback);

    Map_local_pub = nh_pub.advertise<nav_msgs::OccupancyGrid>("map_local", 1);
    ros::Rate rate(10.0);
    printf("localmap operation!\n");

    while( ok() )
    {
        ros::spinOnce();

        if (flag_ScanInfo == 1)
            publish_localmap();

        rate.sleep();
    }
    return 0;
}

void publish_localmap(void)
{
    nav_msgs::OccupancyGrid MapLocal;

    float MapX = Map.info.width;
    float MapY = Map.info.height;

    MapLocal.header = Cur_Header;
    MapLocal.header.frame_id = "world";
    MapLocal.info.width = 150;
    MapLocal.info.height = 150;
    MapLocal.info.origin.orientation.w = 1.0;
    MapLocal.info.resolution = 0.05;
    MapLocal.info.origin.position.x = Cur_Pos_m[0] - ( MapLocal.info.width/2  ) * MapLocal.info.resolution;
    MapLocal.info.origin.position.y = Cur_Pos_m[1] - ( MapLocal.info.height/2 ) * MapLocal.info.resolution;

    for(int j=0;j<MapLocal.info.height;j++)
    {
        for(int i=0;i<MapLocal.info.width;i++)
        {
            MapLocal.data.push_back(0);
        }
    }

    for (int ind=0; ind < ScanAngNum-1; ind++)
    {
        rangeScan = scan->ranges[ind * ScanGap + ScanBias];

        if (isnan(rangeScan) == 0)
        {
            bound_x = Cur_Pos_m[0] + rangeScan * cos( Cur_Att_rad[2] + (ind * ScanAngRange / ScanAngNum - ScanAngRange / 2.0)*D2R );
            bound_y = Cur_Pos_m[1] + rangeScan * sin( Cur_Att_rad[2] + (ind * ScanAngRange / ScanAngNum - ScanAngRange / 2.0)*D2R );

            if (fabs(bound_x - Cur_Pos_m[0]) <  ( MapLocal.info.width/2  ) * MapLocal.info.resolution)
            {
                if (fabs(bound_y - Cur_Pos_m[1]) <  ( MapLocal.info.height/2  ) * MapLocal.info.resolution)
                {
                    bound_Gx = (int)((bound_x - MapLocal.info.origin.position.x) / MapLocal.info.resolution);
                    bound_Gy = (int)((bound_y - MapLocal.info.origin.position.y) / MapLocal.info.resolution);

                    MapLocal.data[bound_Gy*MapLocal.info.width + bound_Gx] = 100;
                }
            }
        }
    }
    Map_local_pub.publish(MapLocal);
    MapLocal.data.clear();
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

