import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FaceDetector
# 安装的 face_recognitione 库
import face_recognition
# 导入 Opencv 库（常用的 视觉 库）
import cv2
# 获取 功能包share目录 的 绝对路径： /share/功能包名字
from ament_index_python.packages import get_package_share_directory
# 使用 OS库 连接 路径（可以避免 路径错误）
import os 
# 导入 CvBridge 类库，用于 OpenCV 图像和 ROS 图像之间的转换
from cv_bridge import CvBridge

# 导入 时间库
import time

# 创建 人脸识别类
class FaceDetectService(Node):          # 继承 Node类
    def __init__(self):
        super().__init__('face_detect_service')     # 调用父类构造函数，设置节点名称
        # 创建服务，服务名称为 'face_detector'，服务类型为 FaceDetector
        self.service_ = self.create_service(FaceDetector,'face_detector',self.detect_face_callback)
        # 实例化 cvbridge 对象，用于图像转换
        self.bridge = CvBridge()
        # 检测人脸 的 两个参数,并将 两个参数 作为 类的属性
        self.number_of_times_to_upsample = 1   # 上采样次数
        self.model = 'hog'                      # 识别人脸的模型
        # 创建 类属性：图像的默认路径。使用 os功能 进行 路径拼接
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/default.jpg')
        # 若 服务启动，则 打印出来
        self.get_logger().info(f'人脸识别服务 已启动！')


    # 定义 创建服务的 回调函数
    # 参数：
    # request: 请求对象，包含请求数据
    # response: 响应对象，用于 返回结果
    # 返回值： response 对象
    def detect_face_callback(self,request, response):
        # 1. 判断 request请求 是否 有 图像数据
        if request.image.data :
            # 如果 有 图像，则 将 request获取的 ros 图像 转换成 opencv 图像
            # 使用 bridge.imgmsg_to_cv2()函数
            cv_image = self.bridge.imgmsg_to_cv2(request.image)

        else:
            # 如果 图像为空，则 使用 默认图像 作为 内容
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info(f'传入的图像 为空，使用默认图像！')
    
        # 此时，cv_image 已经是 OpenCV 图像格式
        
        # 2. 对 opencv 图像 进行 处理
        # 统计 消耗的时间
        start_time = time.time()                # 记录开始时间(time.time() 返回当前时间的时间戳)
        self.get_logger().info(f'加载完成图像，开始 识别！')
        # 在 图片中，寻找 人脸。返回 人脸位置
        face_location = face_recognition.face_locations(cv_image,number_of_times_to_upsample=self.number_of_times_to_upsample,model=self.model)
        # 记录 使用时间
        response.use_time = time.time() - start_time
        # 返回 人脸的个数
        response.number = len(face_location)            # 使用 len() 检测 face_location的数量
        # 返回 人脸位置 的 上下左右 四个坐标
        for top,right,bottom,left in face_location:
            response.top.append(top)            # 将 人脸的 上坐标 添加至 response.top中
            response.right.append(right)        # 将 人脸的 右坐标 添加至 response.right中
            response.bottom.append(bottom)      # 将 人脸的 下坐标 添加至 response.bottom中
            response.left.append(left)          # 将 人脸的 左坐标 添加至 response.left中
        return response                                 # 必须返回 reponse

def main():
    # 初始化 ros2 节点
    rclpy.init()
    
    node = FaceDetectService()  # 创建人脸识别服务 节点

    rclpy.spin(node)    # 保持节点运行，等待请求

    rclpy.shutdown()  # 关闭节点