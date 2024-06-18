#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
// #include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fstream> //文件读入读出
#include <ctime>
#include <cstdlib> //exit
#include <vector>
#include <array>
 
using namespace std;         
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped p;
nav_msgs::Path path;
int global_status;
 
array<double,3> Stringsplit(string str,const char split)
{
      array<double,3> arr;
      int n{};
    istringstream iss(str);    // 输入流
    string token;            // 接收缓冲区
    while (getline(iss, token, split))    // 以split为分隔符
    {
            arr[n] = stod(token);
            n++;
    }
      return arr;
}
void result_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
      actionlib_msgs::GoalStatus status;
      status = msg->status;
      global_status = status.status;
}
int main(int argc,char** argv)
{
      ros::init(argc,argv,"read_path");
      ros::NodeHandle n;
      ros::Publisher pub_path = n.advertise<nav_msgs::Path>("read_path",1000);
      ros::Publisher follow_path = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
      ros::Subscriber result_sub = n.subscribe("/move_base/result",10,result_cb);
      
      ifstream file;
      file.open("/home/zdc/pcd2pgm_ws/src/pcd2pgm/data/gps.txt");
      if(!file.is_open())
      {
            cout<<"can not open the file."<<endl;
            exit(EXIT_FAILURE);
      }
      string str;
      vector<array<double,3>> path_;
      while(getline(file,str))
      {
            auto val = Stringsplit(str,' ');
            path_.emplace_back(val);
      }
      file.close();
 
      ros::Rate r(1);
      path.header.frame_id = "map";
      path.header.stamp = ros::Time::now();
      for(const auto& n : path_)
      {
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = n[0];
            pose.pose.position.y = n[1];
            pose.pose.position.z = 0;
            pose.pose.orientation.w = n[2];
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
 
            path.poses.emplace_back(pose);
            ROS_INFO("( x:%0.6f ,y:%0.6f ,w:%0.6f)",n[0] ,n[1] ,n[2] );
      }
 
      while(ros::ok())
      {
            pub_path.publish(path);
            ros::spinOnce();  
            r.sleep();
      }
      return 0;
}
