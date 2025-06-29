#include "rclcpp/rclcpp.hpp"             // 引入头文件.hpp

     
int main(int argc,char** argv){         // 固定写法，使用args：具体看 learn_args.cpp           

    // 1. ros初始化.
    rclcpp::init(argc,argv);            // 其中，argc和argv可以传递给rclcpp中，可用于修改其它东西。
    
    // 2. 创建node节点,名字为：cpp_node
    auto node = std::make_shared<rclcpp::Node>("cpp_node");   // 创建rclcpp下的node的共享指针。（make_share为共享指针）

    // 自定义：打印日志
    RCLCPP_INFO(node->get_logger(),"hello world!");             // RCLCPP_INFO 为一个 宏定义; 使用node指针 找到get_logger函数

    // 3. 运行节点，循环直到中断
    rclcpp::spin(node);                 // spin是不断循环检测，并处理node事件

    // 4. 关闭节点，清理资源
    rclcpp::shutdown();

    return 0;
}