import rclpy                    # 导入 rclpy库（功能包）
from rclpy.node import Node     # 从 rclpy功能包中的node节点文件 中， 引入 Node类（用于继承）


# 定义 一个类 的声明        【第一个参数一定要是 self】
class PersonNode(Node):                             # 括号中，加入待继承的 Node类
    # 定义 init方法：表示 定义属性（init函数 会在实例化 时，自动被调用）
    def __init__(self,node_name:str,name_value:str,age_value:int) -> None:     # self 是 固定存在的,表示 类自己；name_value 是 str类型（为一个 形参，需要实例化）
        
        # 引入 父类 的init属性
        super().__init__(node_name)

        # self.属性名: 表示 类的属性（只在 类中调用 属性时，用 self.）
        self.name = name_value                  # 表示：将name_value 传入的值，添加至 类的 name属性中，
        self.age = age_value                    #      定义 类 的一个属性的名字为 age，并将 age_value的值 传入

    # 定义 方法：（函数）
    def eat(self,food_name:str):        # 调用时，需要 输入一个变量
        # f 表示：python中的 格式化字符串。使用{ }可以调用 值
        # print(f"{self.name}今天过了自己的{self.age}岁的生日，并且吃了{food_name}")

        # 调用 父类的方法 来 打印字符（使用 self）
        # 因为 父类的方法 已经继承到了该类中，所以直接使用self即可调用该方法                            
        self.get_logger().info(f"{self.name}今天过了自己的{self.age}岁的生日，并且吃了{food_name}")                   


def main():
    rclpy.init()        # 初始化节点,分配资源
    
    # 实例化：创建对象node
    node = PersonNode('jiedian','zyx',25)
    # 使用对象，调用方法
    node.eat('蛋糕')

    rclpy.spin(node)    # 运行节点对象，并收集事件与执行
    rclpy.shutdown()    # 关闭节点，并清理内存
