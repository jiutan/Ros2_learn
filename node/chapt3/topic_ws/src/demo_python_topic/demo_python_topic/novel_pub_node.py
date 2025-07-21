'''
文件类型：节点node
节点名：novel_pub_node.py
功能：实现 下载 小说，并通过 话题 间隔5s 发布一行
'''
import rclpy
from rclpy.node import Node      # 从 rclpy库中的node文件 引入 Node类功能

# 接下来 引入 下载功能包
import requests             # requests下载库

# 导入 消息接口 库
from example_interfaces.msg import String       # string消息接口(也是一个 类，使用时 需创建对象)，用于 发布 字符串

# 队列 库
from queue import Queue                 # 导入 Queue类

# 定义一个类：小说发布节点 类（继承 Node类）
class NovelPubNode(Node):               # 继承 父类Node类
    # 
    '''
    功能：定义 类属性，以及 【创建 类 对象】
    作用：在这里， 创建 类的 对象 
    '''
    def __init__(self,node_name):       # 设置 形参 包含 节点名字
        super().__init__(node_name)     # 从父类super()中，继承 父类的init属性，并 赋予 node_name参数
        # 以上 继承了 Node类
        # 接下来，可以使用Node类中的 成员函数了
        self.get_logger().info(f'{node_name},启动！ ')
        '''
        功能：创建 队列 （用于 将 小说的内容 数组 形成 队列。从头部开始，一行行进行 发布）
        作用：用于 存储 按行分割后的 内容
        语法：self.对象名 = Queue()
        '''
        self.novel_queue_ = Queue()              # 需 放在 调用 该队列 函数 之前
        '''
        功能：创建 发布者
        来源：ros2中 node库里 继承来的 
        语法：self.话题发布的对象名 = self.create_publisher(patameters)
        参数：
            1. msg_type：消息接口(消息类型) （使用 string消息接口，用于 发布 字符串 信息）【在 消息接口库中】
            2. topic： 话题的名字 （随便 取一个 话题的名字）
            3. qos_profile： qos的服务质量配置 （只需 给一个 数字：表示 队列的大小）
        返回值：为 一个 发布者 (需 创建 一个 对象 来 接受)
        '''
        self.novel_publisher_ = self.create_publisher(String,'novel',10)         # 创建 话题的发布者
        '''
        功能：定时 功能
        来源：ros2中 node库里 继承来的
        语法： self.create_timer()
        参数：
            1. timer_period_sec：时间周期(浮点型，单位为s)
            2. callback：回调函数。 （每隔一定的时间周期， 调用 一次 回调函数）
        '''
        self.create_timer(5,self.timer_callback)        # 每5s，进行一次 回调函数
        

    # 成员函数：定义 回调函数（实现： 发布  内容）
    def timer_callback(self):
        # self.novel_publisher.publish()
        # pass                    # 空 函数
        if self.novel_queue_.qsize() > 0:           # 判断是否有内容。若 novel_queue_中 有 内容，则 取内容 放入 line中，等待发布
            line = self.novel_queue_.get()
            '''
            功能：使用 string消息接口 发布 数据
            '''
            msg = String()                          # 1. 先创建 string对象（string也为一个类）。【组装成消息】
            msg.data = line                         # 2. 将 line中存放的内容，复制到 msg.data中（为 string类型）
            '''
            功能：实现 发布
            语法：self.话题发布的对象名.publish(paramater)
            参数：待发布的string对象
            '''
            self.novel_publisher_.publish(msg)
            # 检测:打印内容
            self.get_logger().info(f'发布了：{msg}')


    # 成员函数：下载功能（实现 从url中请求 内容，并进行下载）
    def download(self,url):              
        # 请求，并 下载
        response = requests.get(url)      
        # 下载后的结果 , 使用 utf-8 的编码
        response.encoding = 'utf-8'
        # request.txt
        # 测试：打印 小说 内容的长度
        text = response.text                        # 获取 小说 内容
        self.get_logger().info(f'下载{url},并输出{len(text)}')
        # 分割为 一行一行
        # text.splitlines()             # 返回的 为 一个 数组
        '''
        功能：按行 分割 放入 队列 中
        语法： 
            1. 按行 分割：text.splitlines()
            2. 放入 队列中：self.队列对象名.put(line)
        '''
        for line in text.splitlines():
            self.novel_queue_.put(line)         # 将 分割后的内容 每一行 放入 novel_queue_中

     


# 定义 主函数
def main():
    rclpy.init()                            # 初始化
    node = NovelPubNode('novel_pub')        # 将 类实例化，创建对象
    
    # download novel
    node.download('http://0.0.0.0:8000/novel1.txt')

    rclpy.spin(node)                        # 运行 node对象
    rclpy.shutdown()                        # 关闭 对象