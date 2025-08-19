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

# 导入 SetParameters服务
from rcl_interfaces.srv import SetParameters
# 导入 SetParameters内的 Parameter消息接口
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType   # ParameterType 是 参数类型的规定

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

    '''
    函数 功能： 调用服务，修改 参数值
    '''
    def call_set_parameters(self,parameters):
        # 1. 创建 参数更新 客户端
        update_param_client = self.create_client(SetParameters,'/face_detect_service/set_parameters')  # 名字为 列表中的
        # 2. 等待 参苏更新 服务端 上线
        while update_param_client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待 参数更新 服务端上线')
        # 3. 创建 request
        request = SetParameters.Request()
        request.parameters = parameters         # 设置 请求的 参数信息 Parameter。可以在 interface中查看
        # 4. 调用 服务端 更新 参数（异步调用），返回给 future
        future = update_param_client.call_async(request)
        # 5. 等待 参数服务器 返回 响应结果
        rclpy.spin_until_future_complete(self,future)                       # 等待 服务端 返回 响应   
        # 6. 创建 response 为 响应结果
        response = future.result()
        return response                         # 到上一级 处理 response

    '''
    函数功能 ： 更新 检测的模型
        参数： model = 默认值
    函数实现：根据 传入的model，构造Parameters，然后再调用 call_set_parameters 来 更新 服务端参数
    
    parameter消息 参数：
        name 
        parameterValue(复合参数，嵌套)： 需 创建一个parameterValue 再对其 进行赋值。【其中，parameterValue 也是个 消息接口，需导入库】

    '''
    # 消息内容 可 通过 ros2 interface show rcl_interfaces/srv/SetParameters 查看
    def update_detect_node(self,model='hog'):
        # 1. 创建 Parameter消息接口 参数对象
        param = Parameter()                     # 调用 Parameter() 消息接口
        param.name = 'model'                    # 设置 name 参数
        # 2. 创建 param_value消息接口 对象，并且对 param_value 的 参数 进行赋值
        param_value = ParameterValue()          # 调用 parameterValue() 消息接口
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING       # 系统 固定的 参数类型
        # 3. 将 value 参数 设置为 param_value消息接口
        param.value = param_value
        # 4. 请求 更新参数
        response = self.call_set_parameters([param])            # 函数 参数 为 一个 数组
        # 其 返回值 是一个 SetParametersResult[]消息接口 数组，所以 需要 遍历 来 获取 内容
        for result in response.results:
            if result.successful:
                self.get_logger().info(f'设置参数结果：{result.successful}与{result.reason}')


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
            # self.show_response(response) 不用参数，则取消 注释
        
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

    node.update_detect_node('hog')

    node.send_request()         # 调用 send_request

    node.update_detect_node('cnn')

    node.send_request()  

    rclpy.spin(node)  # 保持 节点 运行

    rclpy.shutdown()  # 关闭 ROS2 客户端    