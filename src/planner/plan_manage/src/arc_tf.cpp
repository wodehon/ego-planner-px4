#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class ArcTfPublisher {
public:
    ArcTfPublisher() {
        // 初始化节点句柄
        nh_ = ros::NodeHandle();
        
        // 订阅里程计话题
        odom_sub_ = nh_.subscribe("/ego/odom", 10, &ArcTfPublisher::odomCallback, this);

        // 创建定时器以固定频率发送变换
        tf_timer_ = nh_.createTimer(ros::Duration(0.02), &ArcTfPublisher::tfCallback, this);
    }

    void spin() {
        // 进入循环，处理回调
        ros::spin();
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 更新变换信息
        odom_trans_.header.stamp = ros::Time::now();
        odom_trans_.header.frame_id = "map";
        odom_trans_.child_frame_id = "base_link";

        odom_trans_.transform.translation.x = msg->pose.pose.position.x;
        odom_trans_.transform.translation.y = msg->pose.pose.position.y;
        odom_trans_.transform.translation.z = msg->pose.pose.position.z;
        odom_trans_.transform.rotation = msg->pose.pose.orientation;

        // 使用标志位来确保仅在有新数据时发送变换
        new_data_available_ = true;
    }

    void tfCallback(const ros::TimerEvent&) {
        if (new_data_available_) {
            // 发送变换
            odom_broadcaster_.sendTransform(odom_trans_);
            new_data_available_ = false; // 重置标志位
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Timer tf_timer_;
    tf::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::TransformStamped odom_trans_;
    bool new_data_available_ = false; // 用于跟踪新数据的标志位
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arc_tf_publisher");

    // 创建 ArcTfPublisher 对象
    ArcTfPublisher arc_tf_publisher;

    // 等待订阅器设置
    ros::Duration(1.0).sleep();

    // 启动循环
    arc_tf_publisher.spin();

    return 0;
}
