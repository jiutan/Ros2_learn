#include "rclcpp/rclcpp.hpp"

// 定义 一个类class
class PersonNode : public rclcpp::Node
{ // 继承 使用 public；继承 rclcpp包 下的 Node类

    // 声明 成员数据（属性）【私有 成员】
private: // 成员声明： 数据类型  成员名      ； 加入 下划线 表示为 私有成员
    std::string name_;
    int age_;

    // 声明 成员函数（方法、构造函数）【共有 函数，可被调用】
public:
    // 设置成员数据，通过初始化列表(onst std::string &node_name)，来显式地调用 父类Node()的 成员数据/属性
    PersonNode(const std::string &node_name, const std::string &name, const int age)
        : Node(node_name) /* 调用 父类 的 构造函数*/ // 将PersonNode获取的node_name，传入 父类Node中
    {
        // 对 成员数据（属性） 进行 赋值: 使用 this -> 属性 = 形参      [this 表示 这个 类]
        this->name_ = name;
        this->age_ = age;
    }

    // 声明 方法
    void eat(const std::string &food_name)
    {

        // 使用 node父类的方法: 使用 this 说明 get_logger()是 继承到该类中的
        RCLCPP_INFO(this->get_logger(),"我是%s,%d岁，爱吃%s", this->name_.c_str(), // name_.c_str() 将 char类型的name 转换成 str类型
                    this->age_, food_name.c_str());                        // name 与 age 都为 私有属性，所以用 this 来调用

    }; // 短函数，可在 类中定义（结尾需加分号；）

}; // 必须有 分号；


int main(int argc,char **argv){

    // 初始化 节点
    rclcpp::init(argc,argv);
    
    // 创建对象:实例化 类
    // make_shared 将 node 变为一个 PersonNode类的指针  【智能指针】
    auto node = std::make_shared<PersonNode>("person_node","zyx",18);     // 将 PersonNode需要的参数(包括父类需要的)，都加上
    // 打印日志
    RCLCPP_INFO(node->get_logger(),"hello c++ node");
    
    // 运行 eat
    node -> eat("鱼香肉丝") ;               // 使用 -> 来 调用 成员函数（方法）

    // 运行 节点
    rclcpp::spin(node);
    // 清理内存
    rclcpp::shutdown();
    
    return 0;
}
