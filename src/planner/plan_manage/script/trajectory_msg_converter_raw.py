#!/usr/bin/env python

import rospy
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from mavros_msgs.msg import PositionTarget

from std_msgs.msg import Float64MultiArray

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        self.pose = PositionTarget()
        self.pose.coordinate_frame = 1
        self.pose.position.x = 0
        self.pose.position.y = 0
        self.pose.position.z = 1
        self.pose.yaw = 0

        self.wp = Float64MultiArray()
        self.wp_pub = rospy.Publisher('rtps/setpoint_raw', Float64MultiArray, queue_size=1)

        self.traj_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        # self.traj_pub = rospy.Publisher('local', PositionTarget, queue_size=1)
        rospy.Subscriber('planning/pos_cmd', PositionCommand, self.fastPlannerTrajCallback)


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

        self.pose.yaw = msg.yaw

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
        

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            self.wp_pub.publish(self.wp)

            self.traj_pub.publish(self.pose)
            rate.sleep()

if __name__ == '__main__':
    obj = MessageConverter()
    obj.run()
