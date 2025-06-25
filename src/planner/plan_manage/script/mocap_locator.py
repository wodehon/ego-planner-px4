#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry # for Fast-Planner
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped

# from std_msgs.msg import Float32MultiArray, Bool

# from scipy.spatial.transform import Rotation as R
# from math import pi

class MocapLocator:
    def __init__(self):
        rospy.init_node('mocap_locator')

        rospy.Subscriber('/vrpn_mocap/nx_6x/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/vrpn_mocap/nx_6x/twist', TwistStamped, self.twist_cb)
        # rospy.Subscriber('/vrpn_client_node/uav/accel', TwistStamped, self.twist_cb)
        # rospy.Subscriber('/reach', Bool, self.hover)
        # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.odom = Odometry()
        # self.pose_pub = rospy.Publisher('/planner', PositionTarget, queue_size=1)

        # rospy.Timer(rospy.Duration(0.01), self.odom_tcb)
        self.odom_pub = rospy.Publisher('/vicon/odom', Odometry, queue_size=1)
        self.pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        

    def pose_cb(self,msg):
        self.pose.header.frame_id = "world"
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = msg.pose.position.x
        self.pose.pose.position.y = msg.pose.position.y
        self.pose.pose.position.z = msg.pose.position.z
        self.pose.pose.orientation.x = msg.pose.orientation.x
        self.pose.pose.orientation.y = msg.pose.orientation.y
        self.pose.pose.orientation.z = msg.pose.orientation.z
        self.pose.pose.orientation.w = msg.pose.orientation.w

        # self.pose_pub.publish(self.pose)

        self.odom.child_frame_id = "base_link"
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "world"
        self.odom.pose.pose.position.x = self.pose.pose.position.x
        self.odom.pose.pose.position.y = self.pose.pose.position.y
        self.odom.pose.pose.position.z = self.pose.pose.position.z
        self.odom.pose.pose.orientation.x = self.pose.pose.orientation.x
        self.odom.pose.pose.orientation.y = self.pose.pose.orientation.y
        self.odom.pose.pose.orientation.z = self.pose.pose.orientation.z
        self.odom.pose.pose.orientation.w = self.pose.pose.orientation.w
        
        self.odom_pub.publish(self.odom)


    def twist_cb(self,msg):
        self.twist.twist.linear.x = msg.twist.linear.x
        self.twist.twist.linear.y = msg.twist.linear.y
        self.twist.twist.linear.z = msg.twist.linear.z
        self.twist.twist.angular.x = msg.twist.angular.x
        self.twist.twist.angular.y = msg.twist.angular.y
        self.twist.twist.angular.z = msg.twist.angular.z

        self.odom.twist.twist.linear.x = self.twist.twist.linear.x
        self.odom.twist.twist.linear.y = self.twist.twist.linear.y
        self.odom.twist.twist.linear.z = self.twist.twist.linear.z
        self.odom.twist.twist.angular.x = self.twist.twist.angular.x
        self.odom.twist.twist.angular.y = self.twist.twist.angular.y
        self.odom.twist.twist.angular.z = self.twist.twist.angular.z



    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            # self.pose_pub.publish(self.pose)

            # print("___test____")

            # self.traj_pub.publish(self.pose)
            # self.update()
            rate.sleep()

if __name__ == '__main__':
    obj = MocapLocator()
    obj.run()
