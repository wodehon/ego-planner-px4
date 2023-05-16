#! /usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv_bridge

# 回调函数处理深度图像消息
def depth_image_callback(msg):
    bridge = cv_bridge.CvBridge()
    
    # 将ROS图像消息转换为OpenCV格式
    cv_image = bridge.imgmsg_to_cv2(msg)
    
    # 获取图像尺寸
    height, width = cv_image.shape[:2]
    
    # 计算中心像素坐标
    center_x = int(width / 2)
    center_y = int(height / 2)
    
    # 获取中心像素的深度值
    depth_value = cv_image[center_y, center_x]
    
    # 打印深度值
    rospy.loginfo("Depth value at center pixel: {}".format(depth_value))

# 初始化ROS节点
rospy.init_node('depth_image_subscriber')

# 订阅深度图像话题
rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)

# 循环等待回调函数触发
rospy.spin()