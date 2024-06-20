#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float32MultiArray, Bool

from scipy.spatial.transform import Rotation as R
from math import pi

class PlanningYaw:
    def __init__(self):
        rospy.init_node('yaw_planning')

        self.pose = PositionTarget()
        
        self.pose.yaw = 0
        self.pose.yaw_rate = 0.5
        
        self.kp = 2 
        self.ki = 0.0
        self.kd = 0
        self.integral = 0.0
        self.last_error = 0.0

        self.discover = Bool()
        self.begin = Bool()
        self.begin.data = False
        self.hover = Bool()
        self.hover.data = False

        rospy.Subscriber('yolov5/error', Float32MultiArray, self.planning)
        rospy.Subscriber('discover', Bool, self.discovercb)
        rospy.Subscriber('/insulator_pub_flag', Bool, self.begincb)
        rospy.Subscriber('/reach', Bool, self.hover)
        # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        self.pose_pub = rospy.Publisher('/planner', PositionTarget, queue_size=1)

        # rospy.Timer(rospy.Duration(0.01), self.yawcal)
        

    def begincb(self,msg):
        self.begin.data = True

    def discovercb(self,msg):
        self.discover.data = msg.data

    def hover(self,msg):
        self.hover.data = True
        self.pose.yaw_rate = 0
        self.pose.yaw = 2.06 #?
        print("---------test-------")


    def yawcal(self, event):
        if self.begin.data and not self.hover.data:
            self.pose.yaw += (self.pose.yaw_rate * 0.01)
            if self.pose.yaw > pi:
                self.pose.yaw -= 2*pi
            if self.pose.yaw < -pi:
                self.pose.yaw += 2*pi
        # if self.hover.data:
        #     self.pose.yaw_rate = 0


    def planning(self, msg):
        # if self.trig == 0:
        #     self.trig = msg.data
        # if msg.data == 0:
        #     return
        # if msg.data > 0:
        #     self.pose.yaw = self.pose.yaw + 0.005
        # elif msg.data < 0:
        #     self.pose.yaw = self.pose.yaw - 0.005
        error = msg.data
        
        yawdot = self.pid(error)
        self.pose.yaw_rate = yawdot

        #cal
        if self.begin.data and not self.hover.data:
            self.pose.yaw += (self.pose.yaw_rate * 0.01)
            if self.pose.yaw > pi:
                self.pose.yaw -= 2*pi
            if self.pose.yaw < -pi:
                self.pose.yaw += 2*pi
        if self.hover.data:
            self.pose.yaw_rate = 0
            # self.pose.yaw = 2.06


        print("____yawdot=" + str(yawdot))
        print("yaw=" + str(self.pose.yaw))

    
    # def pose_callback(self,msg):
    #     # [msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]
    #     eular = quaternion2euler([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    #     self.pose.yaw = eular[2]

    
    def pid(self,error):
        p = self.kp * error[0]

        self.integral += error[0]
        i = self.ki * self.integral

        derivative = (error[0] - self.last_error) / error[1]
        d = self.kd * derivative
        self.last_error = error[0]

        return p + i + d
        

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            self.pose_pub.publish(self.pose)

            # print("___test____")

            # self.traj_pub.publish(self.pose)
            # self.update()
            rate.sleep()


def quaternion2euler(quaternion):
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=True)
        return euler

if __name__ == '__main__':
    obj = PlanningYaw()
    obj.run()
