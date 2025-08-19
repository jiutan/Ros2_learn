#include<rclcpp/rclcpp.hpp>
#include"custom_interfaces/srv/patrol.hpp"
// 定义 命名空间
using Patrol = custom_interfaces::srv::Patrol;

#include<chrono>
// 使用 字面量
using namespace std::chrono_literals;  // 使用 chrono 命名空间下的 字面量 后缀.可以使用 10s, 1s, 100ms 等等

// 使用 ctime 产生 随机数
#include<ctime>

// 创建 客户端 类
class PatrolClientNode:public rclcpp::Node
{
private:
    // 声明/创建 定时器 智能指针 成员
    rclcpp::TimerBase::SharedPtr timer_;

    // 声明/创建 客户端 智能指针 成员(模板类:消息接口名字)
    rclcpp::Client<Patrol>::SharedPtr Patrol_client_;
    
public:
    // 构造函数 并 继承 Node类
    /**
     *  构造函数：初始化 节点类 参数列表，把 node_name 传给 Node父类，完成 节点类创建
     *  注意：这里的 node_name 是 节点名称
     *  其中：冒号:Node(node_name)作用为 成员初始化列表（member initializer list）。其 显示地调用 Node类的 构造函数，并将 node_name 作为参数传递。
     *     *  这样做的好处是：可以在构造函数体内使用 Node类 的 成员函数和属性。
     *     *  rclcpp::Node 没有无参构造函数（必须提供节点名称）。
     */ 
    explicit PatrolClientNode(const std::string &node_name):Node(node_name){
        
        // 初始化 客户端 智能指针(node下的 create_client<消息接口>(服务名))
        Patrol_client_ = this -> create_client<Patrol>("patrol");       // 客户端名字 需与 服务端一致

        // 初始化 随机数 种子
        srand(time(NULL));          // 根据 当前时间 产生随机数

        // 初始化 定时器 【 node下 create_wall_timer(周期，回调函数，回调组=有默认值可以不传入)  】, 每隔 10s 发送一次
        // 回调函数 为 lambda表达式
        timer_ = this -> create_wall_timer(10s,[&]()->void{ 
            // 1. 循环 检查 客户端 是否 上线
            while (! Patrol_client_ -> wait_for_service(1s))  // 等待服务上线，超时时间为1秒
            {
                if(! rclcpp::ok()){                   // 检查 rclcpp 是否正常

                    RCLCPP_ERROR(this->get_logger(),"等待服务上线过程中，rclcpp 挂了，退出程序");
                    return;                         // 退出程序
                }    
                // 若 客户端未上线，则打印日志
                RCLCPP_INFO(this->get_logger(),"等待 服务上线中....");
                
            }
            // 2. 构造 请求 对象。创建 request 共享指针
            auto request = std::make_shared<Patrol::Request>();
            // 对其 移动的目标点 进行赋值（随机）
            request -> target_x = rand() % 15;      // 随机数 在 0～15之间
            request -> target_y = rand() % 15;      // 随机数 在 0～15之间

            // 打印日志
            RCLCPP_INFO(this->get_logger(),"准备好目标点%f,%f",request->target_x,request->target_y);

            // 3. 发送 请求（异步async 发送请求）
            // 回调函数： 服务端 响应 请求后，进入 回调函数
            /*  
                异步发送时：
                * 会 返回 一个 rclcpp::Client<Patrol>::SharedFuture 对象
                * 在 rclcpp::Client<Patrol>::SharedFuture 对象 里 存放 response
             */
            this->Patrol_client_->async_send_request(request,
                [&](rclcpp::Client<Patrol>::SharedFuture result_future)->void{
                    // 获取 response 回应
                    auto response = result_future.get();
                    // 判断 response 是否 成功
                    if(response -> result == Patrol::Response::SUCCESS){
                        RCLCPP_INFO(this->get_logger(),"请求 巡逻目标点 成功");
                    }
                    if(response -> result == Patrol::Response::FAIL){
                        RCLCPP_INFO(this->get_logger(),"请求 巡逻目标点 失败");
                    }
            });
        });
        
    }
};








int main(int argc,char *argv[]){

    // 初始化 ROS2
    rclcpp::init(argc,argv);  

    // 创建 节点
    auto node = std::make_shared<PatrolClientNode>("Patrol_client");

    // 运行节点
    rclcpp::spin(node);

    // 关闭节点
    rclcpp::shutdown(); 

    return 0;
}