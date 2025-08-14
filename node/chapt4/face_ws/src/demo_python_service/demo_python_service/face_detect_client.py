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
class FaceDetectClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')  # 初始化 节点 名称
        # 创建 cv_bridge 对象 属性
        self.bridge = CvBridge()
        # 创建 图像 路径 属性
        self.image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/test1.jpg')
        # 打印 内容
        self.get_logger().info('人脸服务客户端 已启动')
        # 创建 客户端 对象 属性
        self.client = self.create_client(FaceDetector,'face_detector')
        # 创建 图像 对象属性
        self.image = cv2.imread(self.image_path)  # 读取 图像


    # 定义 发送请求 函数
    def send_request(self):

        # 1. 判断一下，服务端 是否 在线
        while self.client.wait_for_service(timeout_sec=1.0) is False:       # 循环检测 服务端是否在线，1s中等待时间
            # 如果 是 False，说明 服务端不在线 ，则 打印 日志
            self.get_logger().info('等待 服务端上线！')

        # 2 如果 在线，则 构造 Request
        # 注意：【  消息接口 在 编译时，会 自动生成 Request类 和 Response 类    】
        request = FaceDetector.Request()                                    # 创建 请求对象
        # 对 request内容 进行 赋值
        request.image = self.bridge.cv2_to_imgmsg(self.image)               # 将 cv2 图像 转换为 ROS 图像 消息

        # 3. 发送 请求，并 等待 处理完成
        # 其 返回 值 是 一个 future 对象。（当前 还没有包含响应结果，需要 等待 服务端处理完成，才会把结果 放入 future中）
        future = self.client.call_async(request)                            # 创建 一个 服务请求， 并且 异步asyncronously 的 获取结果   
        # 等待 服务端 处理完成
        while not future.done():                                            # 如果 future 还没有完成
            time.sleep(1.0)                                                 # 休眠 当前线程 1s ，等待 服务处理完成。
        # （  需要 开启 多线程 ， 否则 当前线程 无法 再接收 来自服务端的 返回，导致 永远没有办法 future.done() ）
        # 4. rclpy.spin_until_future_complete()方法：会 在后台，一边查看futur是否完成，一边接收结果
        # self 本身 就表示 node 对象
        #rclpy.spin_until_future_complete(self,future)                       # 等待 服务端 返回 响应                                       
        def result_callback(result_future):
             # 5. 获取 响应 结果
            response = result_future.result()                                          # 获取 结果，返回给 response 响应 
            self.get_logger().info(f('接收到 响应，共检测到{response.number}张人脸，耗时{response.use_time}s'))
            # 6. 对 响应 进行 处理
            self.show_response(response)
        
        future.add_done_callback(result_callback)

       

    # 定义 响应结果 显示 函数：对 服务端返回的结果 进行 展示
    def show_response(self,response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            # 绘制 人脸 识别框
            cv2.rectangle(self.image,(left,top),(right,bottom),(255,0,0),4)
        # 显示 人脸框
        cv2.imshow('Face Detect Result',self.image)
        # 也会 阻塞线程，导致 spin 无法正常运行（在本例中，只调用一次，不会有问题）
        cv2.waitkey(0)  



# main函数
def main():
    rclpy.init()  # 初始化 ROS2 客户端

    node = FaceDetectClient()  # 创建 客户端 节点

    node.send_request()         # 调用 send_request

    rclpy.spin(node)  # 保持 节点 运行

    rclpy.shutdown()  # 关闭 ROS2 客户端    