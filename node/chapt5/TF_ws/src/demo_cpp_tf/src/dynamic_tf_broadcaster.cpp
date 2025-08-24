#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/transform_stamped.hpp"
#include"tf2/LinearMath/Quaternion.hpp"                  // 提供 四元数 tf2::Quaternion类
#include"tf2_geometry_msgs/tf2_geometry_msgs.hpp"        // 消息类型 转换函数： 将 内容 转换成 消息接口
#include"tf2_ros/transform_broadcaster.h"                // 坐标 广播器 类

#include "chrono"
using namespace std::chrono_literals;

class TFBroadercast:public rclcpp::Node
{
private:
    // 在 私有成员内部 声明 对象
    // 声明 静态广播器 智能指针 对象
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    // 声明 定时器 对象
    rclcpp::TimerBase::SharedPtr timer_; 

public:
    // 构造函数 编写
    TFBroadercast():Node("tf_broadcaster")
    {
        // 在 构造函数中 ，对 声明的 对象 进行 实例化
        this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);      // this 为 当前节点，需要 使用节点在类内部 创建 发布者
        // 使用 定时器 不停 地 发布
        timer_ = this->create_wall_timer(100ms,std::bind(&TFBroadercast::publish_tf,this)); // 使用 函数包装器 写 定时器
    }

    /**
     * 编写 发布 成员方法
     * 方法：
     *      1. 需要 定义一个 消息接口
     *      2. 对 消息接口 赋值
     */
    void publish_tf(){
        // 1. 定义 消息接口对象
        geometry_msgs::msg::TransformStamped transform;
        // 2. 对 消息接口 赋值  [具体格式 需 查看 消息接口定义]
        transform.header.stamp = this->get_clock()->now();      // 使用 当前时间 作为 时间辍
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        
        // 对 消息接口 的 旋转部分 进行赋值
        //（1）定义一个 四元数：使用 tf2::Quarternion类
        tf2::Quaternion q;
        //（2）设置 四元数 的 内容： q.setRPY(x的弧度,y的弧度,z的弧度) 
        q.setRPY(0.0,0.0,30 * M_PI/180.0);                        // 弧度值 = 角度值 * M_PI / 180
        //（3）赋值
        transform.transform.rotation = tf2::toMsg(q);           // tf2::toMsg(q) 可以将 四元数q 转换成 tranform下的rotation
        
        // 3. 使用 广播器 发布 transform 对象,发送出去
        this->broadcaster_->sendTransform(transform);
    }
};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<TFBroadercast>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}


