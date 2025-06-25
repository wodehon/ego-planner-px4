#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTF
{
public:
    OdomToTF()
    {
        odom_sub_ = nh_.subscribe("/vicon/odom", 10, &OdomToTF::odomCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.02), &OdomToTF::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped odom_to_base_;
    bool has_odom_data_ = false;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_to_base_.header.stamp = msg->header.stamp;
        odom_to_base_.header.frame_id = "map";
        odom_to_base_.child_frame_id = "base_link";
        odom_to_base_.transform.translation.x = msg->pose.pose.position.x;
        odom_to_base_.transform.translation.y = msg->pose.pose.position.y;
        odom_to_base_.transform.translation.z = msg->pose.pose.position.z;
        odom_to_base_.transform.rotation.x = msg->pose.pose.orientation.x;
        odom_to_base_.transform.rotation.y = msg->pose.pose.orientation.y;
        odom_to_base_.transform.rotation.z = msg->pose.pose.orientation.z;
        odom_to_base_.transform.rotation.w = msg->pose.pose.orientation.w;
        has_odom_data_ = true;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (!has_odom_data_) return;
        odom_to_base_.header.stamp = ros::Time::now(); // 更新时间戳
        tf_broadcaster_.sendTransform(odom_to_base_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_tf");
    OdomToTF odom_to_tf;
    ros::spin();
    return 0;
}
