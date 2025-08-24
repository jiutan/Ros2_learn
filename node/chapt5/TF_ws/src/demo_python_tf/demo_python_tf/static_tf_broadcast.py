import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster          # 静态TF坐标 广播器 类
from geometry_msgs.msg import TransformStamped          # 消息接口 库 中 的 类
from tf_transformations import quaternion_from_euler    # 欧拉角 转 四元数 函数
import math                                             # 需要用到 math库 中的 角度转弧度的 函数

class StaticTFBroadcast(Node):      # 继承 Node类
    # pass
    def __init__(self):
        super().__init__('static_tf_broadcaster')        # 继承 父类 init函数，用来给 节点 其名字
        # 如何 发布静态坐标变换？   - 使用 静态坐标广播器 对象 -> 创建 该对象
        self.static_broadcaster_ = StaticTransformBroadcaster(self)         # 内部 需要创建 发布者 发布内容，需要Node=self
        self.publish_static_tf()

    '''
    发布 静态TF：

    原理：使用 消息接口 发送 坐标

    发布 的 方法：使用 消息接口对象（TransformStamped）
    '''
    def publish_static_tf(self):
        '''
        实现：从 base_link 到 camera_link 的 坐标转换
        '''
        # 1. 创建 消息接口对象：用于 发送 坐标变换
        transform = TransformStamped()
        # 2. 给 该 消息接口的内容 赋值
        transform.header.frame_id = 'base_link'                     # 父类 坐标系 的 id
        transform.header.stamp = self.get_clock().now().to_msg()    # 获取 时间辍（常用）
        transform.child_frame_id = 'camera_link'                    # 子类 坐标系 的 id
        # 平移坐标系 设置
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6
        # 旋转坐标系 设置： 将 角度值 -> 弧度值 -> 四元数
        # （1） 将 角度值 转化成 弧度值
        radian_x = math.radians(180)
        # （2） 将 x,y,z 的 弧度值 转换为 四元数(只接受 弧度值)【其 返回值 为 元组: q = [X,Y,Z,W] 】
        q = quaternion_from_euler(radian_x,0,0)
        # （3） 对 旋转坐标系 欧拉角 赋值
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        # 3. 静态坐标系 发布 出去：
        self.static_broadcaster_.sendTransform(transform)           # 使用 StaticTransformBroadcaster
        self.get_logger().info(f'发布静态TF：{transform}')

# main函数
def main():
    # 1. ros初始化
    rclpy.init()
    # 2. 创建 节点类
    node = StaticTFBroadcast()
    # 3. 运行 节点
    rclpy.spin(node)
    # 4. 关闭 节点
    rclpy.shutdown()