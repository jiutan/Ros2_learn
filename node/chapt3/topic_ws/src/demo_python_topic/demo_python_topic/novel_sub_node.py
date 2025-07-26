import espeakng

import rclpy
from rclpy.node import Node

# 导入 string消息接口库 与 队列库
from example_interfaces.msg import String
from queue import Queue

# 因为 读小说 需要单独使用 一个线程（ros2为单线程）
import threading            # 导入线程库，来 创建一个线程

# 导入 时间库
import time                 # 导入 时间库，来让 线程 休眠

class NovelSubNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name},启动')
        # 接下来，在 __init__中：定义 对象
        '''
        创建 队列 对象： slef.对象 = Queue()
        '''
        self.novel_queue_ = Queue()
        '''
        创建 订阅者 对象：self.对象 = Node.create_subscription(parameter)
        参数：
            1. msg_type消息类型（消息接口）
            2. topic话题： 需与 发布者 发布的 话题名字 一致，才能够 订阅的到
            3. callback回调函数： 订阅的时候 需拿取数据。 需 通过回调函数 来 告诉系统 数据接收到了（类似 中断）
            4. qos_profile：保持与 发布者 一致即可
        '''
        # 用 成员对象 来接收 创建 订阅者
        self.novel_subsriber_ = self.create_subscription(String,'novel',self.novel_callback,10)
        '''
        1. 创建 多线程 对象：self.线程对象 = threading.Thread(target=self.使用线程的函数名)
        2. 启动线程：self.线程对象.start()
        '''
        self.speech_thread_ = threading.Thread(target=self.speake_thread)
        # python的线程无法 自己启动，需 启动线程
        self.speech_thread_.start()             # 跳转到 speak_thread成员函数 中，运行


    # 在 类 中 定义 回调函数：有消息时，则 会调用 该函数
    # 发布者 发布一个 msg；订阅者 就接收一个 相同内容的msg（ros2后台完成）
    def novel_callback(self,msg):           # msg 为 novel_pub_node.py中的 msg=String() 同一个
        # 将 小说 放入队列中（put）
        self.novel_queue_.put(msg.data)     # msg本身 为 string对象，我们只需要 msg中的data成员

    # 使用 线程 的 函数：用于 语音读书
    def speake_thread(self):
        # 1. 先 创建一个 speaker（实例化Speaker类）
        speaker = espeakng.Speaker()        # 生成 一个 Speaker类的对象 叫 speaker
        # 2. 设置 语音 的 语言
        speaker.voice = 'zh'
        # 3. 检测 当前上下文 是否正常ok
        while rclpy.ok():
            # 4. 若正常，则 判断 队列大小是否不为0（是否接收到消息）
            if self.novel_queue_.qsize() > 0:
                # 5. 若 队列中 有内容，则 从 队列中 取出 文字 存入 text中
                text = self.novel_queue_.get()
                # 6. 存入 文字后，打印出来 查看 是否收到
                self.get_logger().info(f'朗读：{text}')
                # 7. 调用 语音 进行 读 text中的 内容
                speaker.say(text)
                # 8. 等待 说完
                speaker.wait()      # 说完后，才可 读下一句
            else:   
                # 5. 关键点：若 队列中 没有内容， 则 需 让 当前的线程 休眠【使用 time库】（降低CPU功耗）
                time.sleep(1)                   # 休眠 1s。（来 降CPU功耗 ）




def main():
    rclpy.init()
    node = NovelSubNode('novel_sub')
    rclpy.spin(node)
    rclpy.shutdown()
