#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float64MultiArray, Int8

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        self.pose = PositionTarget()
        self.pose.coordinate_frame = 1
        self.pose.position.x = 0
        self.pose.position.y = 0
        self.pose.position.z = 8 # 10
        self.pose.yaw = 0.4

        self.wp = Float64MultiArray()
        self.wp_pub = rospy.Publisher('rtps/setpoint_raw', Float64MultiArray, queue_size=1)

        self.traj_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        # self.traj_pub = rospy.Publisher('local', PositionTarget, queue_size=1)
        rospy.Subscriber('planning/pos_cmd', PositionCommand, self.fastPlannerTrajCallback)
        rospy.Subscriber('yolov5/tuning', Int8, self.tuning)
        rospy.Subscriber('/planner', PositionTarget, self.yawCallback)
        # rospy.Subscriber('move_base_simple/goal', PositionTarget, self.yawCallback)
        self.trig = 0
        self.yaw = 0.0
        self.yawtrigger = False


    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        
        self.pose.position.x = msg.position.x
        self.pose.position.y = msg.position.y
        self.pose.position.z = msg.position.z
        
        self.pose.velocity.x = msg.velocity.x
        self.pose.velocity.y = msg.velocity.y
        self.pose.velocity.z = msg.velocity.z

        self.pose.acceleration_or_force.x = msg.acceleration.x
        self.pose.acceleration_or_force.y = msg.acceleration.y
        self.pose.acceleration_or_force.z = msg.acceleration.z

        # 仿真exp2注释
        # self.pose.yaw = msg.yaw 
        # if self.yawtrigger:
        #     self.pose.yaw = self.yaw

        self.wp.data = []

        self.wp.data.append(msg.position.x) 
        self.wp.data.append(msg.position.y) 
        self.wp.data.append(msg.position.z) 

        self.wp.data.append(msg.velocity.x) 
        self.wp.data.append(msg.velocity.y) 
        self.wp.data.append(msg.velocity.z) 

        self.wp.data.append(msg.acceleration.x) 
        self.wp.data.append(msg.acceleration.y) 
        self.wp.data.append(msg.acceleration.z) 

        self.wp.data.append(msg.yaw) 
        self.wp.data.append(msg.yaw_dot) 

        # self.wp_pub.publish(self.wp)
        # self.traj_pub.publish(self.pose)

    
    def yawCallback(self, msg):
        # 仿真exp1注销
        self.pose.yaw = msg.yaw
        self.pose.yaw_rate = msg.yaw_rate
        return


    def tuning(self, msg):
        if self.trig == 0:
            self.trig = msg.data
        if msg.data == 0:
            return
        if msg.data > 0:
            self.pose.yaw = self.pose.yaw + 0.005
        elif msg.data < 0:
            self.pose.yaw = self.pose.yaw - 0.005
        

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            self.wp_pub.publish(self.wp)

            self.traj_pub.publish(self.pose)
            # self.update()
            rate.sleep()

if __name__ == '__main__':
    obj = MessageConverter()
    obj.run()
