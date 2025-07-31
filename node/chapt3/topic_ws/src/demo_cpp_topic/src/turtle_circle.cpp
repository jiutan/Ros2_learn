#include"rclcpp/rclcpp.hpp"

// 导入 话题接口 的 文件
#include"geometry_msgs/msg/twist.hpp"   

// 导入 chrono 文件 中的 时间
#include<chrono>

// C++14的特性。引入这个命名空间后，你就可以直接写 1000ms 来表示1000毫秒，而不是写 std::chrono::milliseconds(1000)，让代码更简洁易读。
using namespace std::chrono_literals;       

/*      第一步： 自己创建一个 C++ 节点类（需继承Node类，继承后 该类 就拥有了 创建发布者、订阅者、定时器等所有节点功能的能力。）
*       为什么使用类封装：在面向对象的ROS2编程中，将一个节点的所有功能（发布者、订阅者、服务器等）都封装在一个类里是非常常见的做法
*/
class TurtleCircleNode:public rclcpp::Node          // 继承 rclcpp 空间下的 Node类
{
private:
    /*  在 私有区域 中 声明 成员变量
        作用：用于 创建的 各类对象 
        注意：只能被 类的内部成员函数 访问
    */

    /*
    *    作用：创建/声明 一个 发布者 的 智能指针。    发布者是用来向特定话题发送消息的对象
    *    rclcpp::Publisher：这是ROS2中 发布者 的 类型。
    *    <> ：模板参数，    指定了这个发布者要发送的 消息类型/话题接口。
    *        Twist：消息通常用来表示物体的 线速度 和 角速度。
    *    ::SharedPtr：智能指针        【在ROS2中，几乎 所有对象 都用 智能指针 来 管理，它可以自动处理内存释放，避免内存泄漏】
    */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;             // publisher_ 就是我们给这个智能指针起的名字（下划线结尾是常见的成员变量命名习惯）。

    // 创建 定时器 对象的智能指针：
    rclcpp::TimerBase::SharedPtr timer_;

public:
    // 构造函数
    // explicit 防止 创建类对象 时 使用 隐式转换（如： 类名 类对象 = 实参）
    //  : Node(node_name)： 初始化列表。 在执行 该构造函数前，调用父类 rclcpp::Node 的 构造函数，并把 传入的节点名称 node_name 传递给 父类，从而完成 节点的命名
    explicit TurtleCircleNode(const std::string& node_name):Node(node_name){        

        // 通过 智能指针 创建 发布者
        // create_publisher() 为 Node类中的 成员函数（继承来的）： 专门用于 创建发布者。其 返回的是：发布者的智能指针
        /**
         * publisher_ = this -> create_publisher<消息接口 模板>("话题名称"，10); 
         * 模板：指定 话题消息 的 类型。（需要与 声明 的 一致）
         * 话题名称： 节点 将会向这个话题 发布消息，turtlesim节点会订阅这个话题来获取控制指令。
         * 10：这是服务质量（QoS）设置中的队列大小 (queue size)。如果发布消息的速度过快，而网络来不及处理，最多可以有10条消息在队列里排队等待发送。
         */
        publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);          // 向 /turtle1/cmd_vel 话题 发送 Twist类型 的 消息
        
        /**
         *  通过 智能指针对象 创建 定时器 
         *  使用 create_wall_timer(period,callback)  （继承rclcpp::Node类）    用于 创建 定时器
         *  模板：无
         *  参数：       1. period：周期。【std::chrono::duration】     
         *              2. callback：回调函数。【使用 绑定的成员函数 作为 回调函数】
         *              3. this
         *  功能：间隔一段时间 调用 回调函数

         *  std::bind(&TurtleCircleNode::timer_callback,this) 语句：
         *    std::bind(...)：bind 函数将成员函数和它的对象实例 “绑定” 在一起，生成一个可以 被直接调用 的 新函数对象。定时器 需要的就是这样一个 可以直接调用的对象。
         *    &TurtleCircleNode::timer_callback：这是 要定时调用 的 回调函数 的 地址。它是一个成员函数。
         *    this：因为 timer_callback 是一个成员函数，它需要知道应该在哪个具体的对象（实例）上执行。this 就代表“当前这个对象”。 
         */
        timer_ = this -> create_wall_timer(1000ms,std::bind(&TurtleCircleNode::timer_callback,this));
    }
    
    /***
     *  定义 成员函数 作为 回调函数
     *  功能：向外 发送消息
     */
    void timer_callback(){

        // 1. 创建 消息对象msg 为 消息接口
        auto msg = geometry_msgs::msg::Twist();            // 创建 一条 Twist 类型 的 消息

        // 2. 对 消息对象 进行赋值
        // Twist 消息包含 linear（线速度）和 angular（角速度）两个部分。
        msg.linear.x = 1.0;             
        msg.angular.z = 1.0;

        // 3. 使用我们之前创建的发布者，将 填充好的消息 发送出去 。
        // ->publish(msg)：调用发布者的 publish 方法，将 msg 对象发布到话题 /turtle1/cmd_vel 上。任何订阅了这个话题的节点（比如 turtlesim）都会收到这条消息
        publisher_ -> publish(msg);         // publisher_ 为 指针 ；publish()为 方法

    }
};

int main(int argc,char *argv[]){

    rclcpp::init(argc,argv);

    auto node = std::make_shared<TurtleCircleNode>("turtle_circle");// 创建节点 的 共享指针对象

    rclcpp::spin(node);         // 运行节点

    rclcpp::shutdown();

    return 0;

}

