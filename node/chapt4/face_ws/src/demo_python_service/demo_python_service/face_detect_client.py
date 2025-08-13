'''
    编写 客户端 的 服务程序
'''
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FaceDetector

import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
import time

# 定义 客户端 类