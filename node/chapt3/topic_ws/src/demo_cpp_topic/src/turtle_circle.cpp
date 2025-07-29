#include"rclcpp/rclcpp.hpp"

// 导入 话题接口 的 文件
#include"geometry_msgs/msg/twist.hpp"   

// 导入 chrono 文件 中的 时间
#include<chrono>

using namespace std::chrono_literals;       // 使用 该工作空间 可以 直接输入 ms

/*      第一步： 自己创建一个 C++ 节点类（需继承Node类）      */
class TurtleCircleNode:public rclcpp::Node          // 继承 rclcpp 空间下的 Node类
{
private:
    /* 在 私有区域 中 声明下 创建的 各类对象 */

    // 创建 发布者 对象的智能指针：       <>为 传入 消息接口的 模板 类    sharedPtr为指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;             // 发布者 的 智能指针

    // 创建 定时器 对象的智能指针：
    rclcpp::TimerBase::SharedPtr timer_;



public:
    // 构造函数
    // explicit 防止 创建类对象 时 使用 隐式转换（如： 类名 类对象 = 实参）
    explicit TurtleCircleNode(const std::string& node_name):Node(node_name){        

        // 通过 智能指针 创建 发布者
        // create_publisher() 为 Node类中的 成员函数（继承来的）
        /**
         * publisher_ = this -> create_publisher<消息接口 模板>("话题接口名字"，10); 
         * 模板：要创建 哪个 消息接口的发布者
         */
        publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);  
        
        /**
         *  通过 智能指针对象 创建 定时器
         *  使用 create_wall_timer(period,callback)      （无模板）
         *  参数：       1. period：周期。【std::chrono::duration】     
         *              2. callback：回调函数。【使用 绑定的成员函数 作为 回调函数】
         *  功能：间隔一段时间 调用 回调函数
         */
        timer_ = this -> create_wall_timer(1000ms,std::bind(&TurtleCircleNode::timer_callback,this));
    }
    
    /***
     *  定义 成员函数 作为 回调函数
     *  功能：向外 发送消息
     */
    void timer_callback(){

        // 1. 创建 消息对象msg 为 消息接口
        auto msg = geometry_msgs::msg::Twist();

        // 2. 对 消息对象 进行赋值
        msg.linear.x = 1.0;
        msg.angular.z = 1.0;

        // 3. 将 消息对象 发布出去
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

