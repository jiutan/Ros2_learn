#include<rclcpp/rclcpp.hpp>
#include"custom_interfaces/srv/patrol.hpp"
// 定义 命名空间
using Patrol = custom_interfaces::srv::Patrol;

#include<chrono>
// 使用 字面量
using namespace std::chrono_literals;  // 使用 chrono 命名空间下的 字面量 后缀.可以使用 10s, 1s, 100ms 等等

// 使用 ctime 产生 随机数
#include<ctime>

// 导入 消息接口库：为了 修改 其他节点 的 参数
#include"rcl_interfaces/msg/parameter.hpp"
// 添加 消息接口 下 的 嵌套消息接口
#include"rcl_interfaces/msg/parameter_value.hpp"
#include"rcl_interfaces/msg/parameter_type.hpp"
#include"rcl_interfaces/srv/set_parameters.hpp"
using SetP = rcl_interfaces::srv::SetParameters;

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

    /**
     * 创建 客户端，发送 request请求 对象，返回 response结果
     *   * 形参列表：请求对象 的 数据，需 外部传入 进来。所以 其 形参 为 请求部分的数据
     */
    SetP::Response::SharedPtr call_set_parameter(const rcl_interfaces::msg::Parameter &param){
        // 1. 创建 客户端对象
        auto param_client = this->create_client<SetP>("/turtle_control/set_parameters");   // 通信 名字
        // 2. 检测 服务是否上线
        while (!param_client->wait_for_service(1s))
        {
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(),"等待服务上线过程中，rclcpp 挂了，退出程序");
                return nullptr;             // 返回 空指针，并 退出 
            }
            RCLCPP_INFO(this->get_logger(),"等待服务上线中......");
        }
        // 3. 构造 request请求 的 智能指针 对象
        auto request = std::make_shared<SetP::Request>();
        // request中 是有 参数数组Parameter[]:  需将 请求的数据param 添加至 参数数组中 
        request -> parameters.push_back(param);                 // C++数组中，使用 push_back 来给 数组 添加数据

        // 4. 发送请求【同步修改】：发送异步请求，等待请求结果；收到结果后，存入response 并 将其返回
        auto future = param_client -> async_send_request(request);                  // 异步 发送 request请求
        // spin_until_future_complete()方法:
        //      参数：1.节点共享指针get_node_base_interface         2. 发送请求对象
        rclcpp::spin_until_future_complete(this->get_node_base_interface(),future);  
        // 5. 发送请求后，获取 响应结果 放入 response对象中
        auto response = future.get();
        // 6. 返回 结果 指针
        return response;
    }

    /**
     * 【模板】
     * 更新参数K 函数
     * 函数作用：外部 可以 调用该函数 来 更新 某个参数值
     */
    void update_server_param_k(double k){
        // 1. 创建 一个 参数服务 的对象（包含：  name 与 value 两个 参数）
        auto param = rcl_interfaces::msg::Parameter();
        // 2. 给 服务对象 name 赋值
        //      string name 与  ParameterValue value（复合消息接口）
        param.name = "k";
        // 3. 创建 value参数对象 ，其为ParameterValue消息接口 类型 
        auto param_value = rcl_interfaces::msg::ParameterValue(); 
        //  分别给 ParameterValue对象 的 参数 赋值
        // 优点：使用 消息接口 标准化
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;    // 类型为 double消息接口 类型 
        // 由于 是 double类型，可 直接赋值 double_value
        param_value.double_value = k;
        // 4. 将 服务对象 的 value 赋值 param_value
        param.value = param_value;
        // 5. 请求 更新/设置参数 并 将 返回的 结果 进行 处理
        auto response = this->call_set_parameter(param);
        // 判断 返回结果
        if(response == NULL)        // response 为 指针
        {
            RCLCPP_INFO(this->get_logger(),"参数 更新 失败！");
            return;
        }
        // 将返回的结果 进行 遍历处理
        // 其 response内容为 ：SetParametersResult[]数组，其 对象为 results
        //      内部参数为：bool successful 与  string reason
        for(auto result : response -> results){
            if(result.successful == false){
                // successful   是否成功
                // reason       失败的原因（注意使用：c_str() 转换为 C语言下的 字符串类型）
                RCLCPP_INFO(this->get_logger(),"参数 更新 失败！其原因为:%s",result.reason.c_str()); 
            }else{
                 RCLCPP_INFO(this->get_logger(),"参数 更新 成功！"); 
            }
        }
    }
};

int main(int argc,char *argv[]){

    // 初始化 ROS2
    rclcpp::init(argc,argv);  

    // 创建 节点
    auto node = std::make_shared<PatrolClientNode>("Patrol_client");

    // 调用 更新参数函数
    node -> update_server_param_k(4.0);

    // 运行节点
    rclcpp::spin(node);

    // 关闭节点
    rclcpp::shutdown(); 

    return 0;
}