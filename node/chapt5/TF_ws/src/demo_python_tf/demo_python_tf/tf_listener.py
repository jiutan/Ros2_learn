import rclpy
from rclpy.node import Node
import rclpy.time 
from tf2_ros import TransformListener,Buffer            # 坐标监听器，缓冲区
from tf_transformations import euler_from_quaternion    # 四元数 转 欧拉角 函数
import math                                             # 需要用到 math库 中的 角度转弧度的 函数

class TFListener(Node):      # 继承 Node类
    # pass
    def __init__(self):
        super().__init__('Dynamic_tf_broadcaster')        # 继承 父类 init函数，用来给 节点 其名字
        
        self.buffer_ = Buffer()

        self.listener_ = TransformListener(self.buffer_,self)         # 内部 需要创建 监听这 接收信息，需要Node=self
        
        # 创建 定时器 对象,并使用 定时器 定时 获取 tf 变换
        self.timer_ = self.create_timer(1,self.tf_transform)

    def tf_transform(self):
        '''
        功能：定时 获取 坐标关系（实时 查询 坐标关系）
        实现：利用 buffer 查询 坐标关系
        函数：
            lookup_transform('父坐标','子坐标',time,duration)：查询 父坐标 到 子坐标的 TF关系
            参数：       time：     查询的时刻。一般为 当前0时刻：rclpy.time.Time(seconds=0.0)      【Time类】
                        durantion：超时 期间。一般为 1s内的区间：rclpy.time.Duration(seconds=1.0)  【Duration类】

            euler_from_quaternion([x,y,z,w])：将 数组[x,y,z,w] 四元数 转换为 欧拉角(弧度值) 形式
            参数：      [x,y,z,w]：     为 transform.rotation.x/y/z/w
        '''
        try:
            # 1 . 将 查询变换的结果 放入 result中 ： 查询 base_link 到 bottole_link 的 第0时刻（当前时刻）的 坐标变换。若 超时1s，则 退出。
            result = self.buffer_.lookup_transform('base_link','bottle_link',
                                                   rclpy.time.Time(seconds=0.0),
                                                   rclpy.time.Duration(seconds=1.0))
            # 2. 若 查询成功，则 获取 TF 的 内容
            transform = result.transform
            self.get_logger().info(f'平移：{transform.translation}')
            self.get_logger().info(f'旋转：{transform.rotation}')           # 旋转 的 四元数 形式
            # 将 四元数 转换成 欧拉角 形式(传入的 需要是一个 数组格式)
            rotation_eular = euler_from_quaternion(
               [transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
                ])
            
            self.get_logger().info(f'旋转RPY：{rotation_eular}')        # 旋转 的 欧拉角 形式

        # 异常 触发
        except Exception as e:
            self.get_logger().warn(f'获取坐标变换失败，原因{str(e)}')



# main函数
def main():
    # 1. ros初始化
    rclpy.init()
    # 2. 创建 节点类
    node = TFListener()
    # 3. 运行 节点
    rclpy.spin(node)
    # 4. 关闭 节点
    rclpy.shutdown()