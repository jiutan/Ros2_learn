'''
    继承
'''
# 从 person_node文件中，导入PersonNode类
from demo_python_pkg.person_node import PersonNode

# 继承 PersonNode
class WriterNode(PersonNode):           # 此时，WriterNode类 已经继承了 PersonNode类
    # 此时，WriterNode 继承了 name和age 属性，在init中就不用再定义了。只需要再定义 新的属性即可
    def __init__(self,name:str,age:int,book_name:str) -> None:           

        # 引入 父类（super） 的 __init__属性 【在 __init__中继承】
        super().__init__(name,age)

        self.book = book_name

    # 也继承了 PersonNode类 的 方法
        
def main():
    node = WriterNode('zyx',20,'厨师技巧')
    node.eat('月饼')