#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"           // 订阅者 消息接口 库
#include <chrono>

#include "custom_interfaces/srv/patrol.hpp"

using Patrol = custom_interfaces::srv::Patrol;

class TurtleConctrolNode : public rclcpp::Node
{
private:

    // 声明 服务(模板类) 智能指针对象
    rclcpp::Service<Patrol>::SharedPtr patrol_service_;

    // 声明 发布者(模板类) 智能指针 对象 ： 用于 发布 控制 消息
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    // 声明 订阅者(模板类) 智能指针 对象 ： 用于 接收 位置 消息
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscripter_;

    // 设置 目标位置
    double target_x_{1.0};
    double target_y_{1.0};

    // PID控制
    double k_{1.0};             // 比例系数 P

    // 设置 最大线速度
    double max_speed_{3.0};

public:
    /*      控制类 节点 构造函数        */
    // 初始化列表 参数，并且 把 形参node_name 传给 Node父类 ， 完成 节点类的创建
    explicit TurtleConctrolNode(const std::string& node_name):Node(node_name){

        // 初始化 服务对象
        patrol_service_ = this -> create_service<Patrol>("patrol",
            [&](const Patrol::Request::SharedPtr request,Patrol::Response::SharedPtr response)->void{
            // 写 回调函数
            if( (0 < request -> target_x && request -> target_x  < 12.0f) && 
                (0 < request -> target_y && request -> target_y  < 12.0f))                                            // 判断 给的目标值 是否合理
            {// 给 目标位置 设值
                this -> target_x_ = request -> target_x;        // 将 request指针 中的 target_x 赋值给 类属性
                this -> target_y_ = request -> target_y; 
            
                response->result = Patrol::Response::SUCCESS;   // 若满足，则 返回 成功
            }else{
                // 否则，返回 失败
                response->result = Patrol::Response::FAIL;  
            }
            /*  由于 C++中 response 为 指针，所以 只需要 修改 response指针中的内容 即可对 response 赋值， 不需要 返回*/
        });

        // 发布者 智能指针 初始化： 创建 发布者
        velocity_publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);

        // 订阅者 智能指针 初始化： 创建 订阅者
        /**
         * create_subscription<消息接口>(参数)
         *  参数：
         *          topic_name：话题名字
         *          qos：
         *          callback：回调函数。 当 订阅者 收到消息 时，调用 回调函数。     
         *              参数：   this
         *                      占位符:std::placeholders::_1（因为 回调函数 有 1个 形参）
         */
        pose_subscripter_ = this -> create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TurtleConctrolNode::on_pose_received,this,std::placeholders::_1));

    }

    /**
     *  实现：利用 pose当前位置 来 实现 闭环控制 (需 根据 小海龟的位置 与 目的地 的 位置 进行 调整)
     *     
     *  回调函数： on_pose_received(参数)
     *      参数：
     *              收到的数据：其 类型为 收到 数据的 共享指针；共享指针 类型为 消息接口类型turtlesim::msg::Pose
     * 
     *  */        
    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose){      // pose 为 当前的位置

        // 1. 获取 小乌龟 当前的 位置
        auto current_x = pose -> x;
        auto current_y = pose -> y;

        RCLCPP_INFO(get_logger(),"当前：x=%f,y=%F",current_x,current_y);            // 打印 当前的 x和y

        // 2. 计算 当前 海龟位置 跟 目标位置 的 距离差和角度差
        auto distance = std::sqrt(                      // 开平方 得距离
            (target_x_ - current_x )*(target_x_ - current_x ) + (target_y_ - current_y)*(target_y_ - current_y)
        );
        
        auto angle = std::atan2(        // // 反正切 函数 atan2：参数 y ，x
            (target_y_ - current_y),(target_x_ - current_x)) - pose -> theta;   // 目标点角度 跟 当前朝向角度 的差

        // 3. 创建 消息 对象，用于 控制 小海龟 的 移动
        auto msg = geometry_msgs::msg::Twist();     // 创建 一条 Twist类型 的 消息

        // 3. 控制 策略
        if (distance > 0.1)             // 若 距离 大于0.1，才 控制
        {   
            RCLCPP_INFO(get_logger(),"distance = %f",angle);

            //角度差 > 0.2 才 进行 角度的调整
            if(fabs(angle) > 0.2)               // fabs() 为 绝对值函数
            {
                RCLCPP_INFO(get_logger(),"yes");
                msg.angular.z = fabs(angle);    // 调整 旋转旋转
            }else                               // 角度差不大，则 直走 即可
            {
               msg.linear.x = k_ * distance;    // 调整 线速度
            }
        }
        
        // 4. 限制 线速度 最大值
        if(msg.linear.x > max_speed_){
            msg.linear.x = max_speed_;
        }

        // 5. 使用 发布对象指针 发布 消息
        velocity_publisher_ -> publish(msg);

    }
};

int main(int argc,char **argv){

    rclcpp::init(argc,argv);

    auto node = std::make_shared<TurtleConctrolNode>("turtle_control");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

