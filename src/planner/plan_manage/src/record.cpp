/***** 实现路径读取 *****/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h> 
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fstream> //文件读入读出
#include <ctime>
 
using namespace std;
 
void callback_path(const nav_msgs::OdometryConstPtr& msg)
{ 
      //outfile用法同cout,存储形式 1 2 3 
      ofstream outfile;
      outfile.setf(ios::fixed, ios::floatfield);
      outfile.precision(2);
      outfile.open("/home/duan/ego-planner-px4/src/planner/traj_utils/src/gps.txt",std::ios::app);
      outfile<<msg->pose.pose.position.x<<" "<<msg->pose.pose.position.y<<" " <<msg->pose.pose.position.z<<" "<<msg->pose.pose.orientation.w<<" "<<msg->twist.twist.linear.x<<" "<<msg->twist.twist.linear.y<<" "<<msg->twist.twist.linear.z<<" "<<endl;
 
      outfile.close();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_record");
    ros::NodeHandle n;
    ros::Subscriber gps_sub = n.subscribe("/mavros/local_position/odom", 100, callback_path);    
    
    ros::spin();
    return 0;
}
