'''
文件类型：节点node
节点名：Template.py
功能：节点模板
'''
import rclpy
from rclpy.node import Node      # 从 rclpy库中 引入 Node类节点功能

# 定义一个类[为模板]
class Template(Node):                   # 继承父类Node
    # 定义 类属性
    def __init__(self,node_name):       # 设置 形参 包含 节点名字
        super().__init__(node_name)     # 从父类super()中，继承 父类的init属性，并 赋予 node_name参数
        # 以上 继承了 Node类
        # 接下来，可以使用Node类中的 成员函数了
        self.get_logger().info(f'{node_name},启动！ ')

# 定义 主函数
def main():
    rclpy.init()                            # 初始化
    node = Template('template_node')        # 将 类实例化，创建对象.(节点名字 为：template_node)
    rclpy.spin(node)                        # 运行 node对象
    rclpy.shutdown()                        # 关闭 对象