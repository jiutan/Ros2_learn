import rclpy
from rclpy.node import Node         # 从 rclpy库中的node类 导入到 Node中

# 定义 main函数：ROS2节点的基础步骤（python）
def main():
    rclpy.init()                    # 1.初始化工作，分配资源
    node = Node('pyhton_node')      # 2.创建节点的实例对象（创建一个节点名字，将 Node类的 python_node这个实例（对象） 赋值给了 node变量）
    # 日志打印功能
    node.get_logger().info('Hello World')      # 获取并使用get_logger日志管理模块，打印info中的提示
    
    rclpy.spin(node)                # 3.循环的运行节点（只要不打断，不会打断）
    rclpy.shutdown()                # 4.关闭并清理节点（推出后，需清理节点资源）

# 定义模块：如果该模块是主模块（__main__），则运行 main()函数. 用于判断当前脚本是直接运行还是作为模块被导入到其他脚本中执行
if __name__=='__main__':            
    main()