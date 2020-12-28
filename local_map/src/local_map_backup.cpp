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

std::string robot_namespace;
int MapDataAvailable = 0;

ros::Publisher Map_local_pub;
ros::Publisher Mar_tar_pub;
ros::Publisher MarArr_tar_pub;

nav_msgs::OccupancyGrid Map;

visualization_msgs::Marker Target;
visualization_msgs::MarkerArray TargetArray;

void publish_localmap(void);
static void QuaterniontoEuler(float& roll, float& pitch, float& yaw);

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // ROS_INFO("Map Received!");
    Map.header = msg->header;
    Map.info = msg->info;
    Map.data = msg->data;
    MapDataAvailable = 1;
}

void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
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
    //printf("[Cur] %.3f %.3f  [Grid] %d %d\n", Cur_Pos_m[0], Cur_Pos_m[1], Cur_Grid[0], Cur_Grid[1]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map");

    ros::NodeHandle nh_map;
    ros::NodeHandle nh_pos;
    ros::NodeHandle nh_pub;

    robot_namespace = ros::this_node::getNamespace();
    robot_namespace.erase(0, 2);
	
    Subscriber map_sub = nh_map.subscribe("map", 1, map_callback);
    Subscriber vis_sub = nh_pos.subscribe("mavros/vision_pose/pose", 1, pos_callback);

    Map_local_pub = nh_pub.advertise<nav_msgs::OccupancyGrid>("map_local", 1);
    ros::Rate rate(4.0);
    printf("localmap operation!\n");

    while( ok() )
    {
	cout << robot_namespace << "\n";
        if (MapDataAvailable==1)
            publish_localmap();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void publish_localmap(void)
{
    nav_msgs::OccupancyGrid MapLocal;

    float MapX = Map.info.width;
    float MapY = Map.info.height;

    MapLocal.header.stamp = Map.header.stamp;
    MapLocal.header.frame_id = robot_namespace+"map";
    MapLocal.info.width = 200;
    MapLocal.info.height = 200;
    MapLocal.info.origin.orientation.w = 1.0;
    MapLocal.info.resolution = Map.info.resolution;
    MapLocal.info.origin.position.x = Cur_Pos_m[0] - ( MapLocal.info.width/2  ) * MapLocal.info.resolution;
    MapLocal.info.origin.position.y = Cur_Pos_m[1] - ( MapLocal.info.height/2 ) * MapLocal.info.resolution;

    for(int j=0;j<MapLocal.info.height;j++)
    {
        for(int i=0;i<MapLocal.info.width;i++)
        {
            Rel_Grid[0] = Cur_Grid[0] - MapLocal.info.width/2 + i;
            Rel_Grid[1] = Cur_Grid[1] - MapLocal.info.height/2 + j;

            if((Rel_Grid[0] >= Map.info.width) || (Rel_Grid[0] < 0))
            {
                MapLocal.data.push_back(-1);
            }
            else
            {
                  if ((Rel_Grid[1] >= Map.info.height) || (Rel_Grid[1] < 0))
                  {
                      MapLocal.data.push_back(-1);
                  }
                  else
                  {
                      MapLocal.data.push_back(Map.data[Rel_Grid[1]*Map.info.width + Rel_Grid[0]]);
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

