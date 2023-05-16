#!/usr/bin/env python

import rospy
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
# from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Float64MultiArray

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        self.pose = Float64MultiArray()
        # self.pose.coordinate_frame = 1
        # self.pose.position.x = 0
        # self.pose.position.y = 0
        # self.pose.position.z = 1
        # self.pose.yaw = 0

        self.traj_pub = rospy.Publisher('rtps/setpoint_raw', Float64MultiArray, queue_size=1)
        rospy.Subscriber('planning/pos_cmd', PositionCommand, self.fastPlannerTrajCallback)


    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        
        self.pose.data.append(msg.position.x) 
        self.pose.data.append(msg.position.y) 
        self.pose.data.append(msg.position.z) 

        self.pose.data.append(msg.velocity.x) 
        self.pose.data.append(msg.velocity.y) 
        self.pose.data.append(msg.velocity.z) 

        self.pose.data.append(msg.acceleration.x) 
        self.pose.data.append(msg.acceleration.y) 
        self.pose.data.append(msg.acceleration.z) 

        self.pose.data.append(msg.yaw) 
        self.pose.data.append(msg.yaw_dot) 


    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            self.traj_pub.publish(self.pose)
            rate.sleep()

if __name__ == '__main__':
    obj = MessageConverter()
    obj.run()
