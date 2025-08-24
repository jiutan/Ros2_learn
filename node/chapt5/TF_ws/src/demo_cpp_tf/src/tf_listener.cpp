#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/transform_stamped.hpp"
#include"tf2/LinearMath/Quaternion.hpp"                  // 提供 四元数 tf2::Quaternion类
#include"tf2_geometry_msgs/tf2_geometry_msgs.hpp"        // 消息类型 转换函数： 将 内容 转换成 消息接口
#include"tf2_ros/transform_listener.h"                   // 坐标 监听器 类
#include "tf2_ros/buffer.h"                              // 提供 buffer 类 的 库

#include "tf2/utils.h"                                   // 提供了 四元数据 转成 欧拉角 的 库

#include "chrono"
using namespace std::chrono_literals;

class TFListener:public rclcpp::Node
{
private:

    // 声明 监听器 对象
    std::shared_ptr<tf2_ros::TransformListener> listener_;

    // 声明 buffer 对象
    std::shared_ptr<tf2_ros::Buffer> buffer_;

    // 声明 定时器 对象
    rclcpp::TimerBase::SharedPtr timer_; 

public:
    // 构造函数 编写:对 声明的 对象 进行 实例化
    TFListener():Node("tf_listener")
    {   
        // 实例化 buffer 对象：用来 存储TF数据，在 回调函数中 查询
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());         // 需要 时间的记录
        // 实例化 listener，用来 获取/订阅 tf2_ros话题 的 数据，并将 其 放入 buffer 中
        this -> listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_,this);      // 使用 指针
        // 使用 定时器 不停 地 调用 函数 获取 TF坐标
        timer_ = this->create_wall_timer(1s,std::bind(&TFListener::get_transform,this)); // 使用 函数包装器 写 定时器
        
    }

    /**
     * 编写 获取 TF坐标 的方法
     * 方法：
     *      1. 
     *      2. 
     */
    void get_transform(){
        /**
         * 需要 捕获 异常： 使用 C++中的 try-catch 语句，进行 异常捕获
         */
        // 1. 在 buffer 里 查询 坐标关系
        try
        {
            // 查询 坐标关系：超时 1s 中 
            const auto transfer = buffer_ -> lookupTransform(
                "base_link",
                "target_point",
                this->get_clock()->now(),
                rclcpp::Duration::from_seconds(1.0f)        
            ); 
            // 2. 若 成功查询，则 执行 获取 查询transfer结果 存入 translation与rotation
            auto translation = transfer.transform.translation;
            auto rotation = transfer.transform.rotation;            // rotation中 包含 四元数

            // 3. 将 rotation中的 四元数 转化成 欧拉角
            // (1) 先 创建 Y P R 变量，用于 存入 Y P R 的 欧拉角
            double y,p,r;
            // (2) 使用tf2::getEulerYPR(rotation,y,p,r)方法：将 YPRW四元数 转化成 欧拉角,放入 y,p,r 变量中。
            tf2::getEulerYPR(rotation,y,p,r);                       // 参数 为 引用，直接更改 y,p,r
            
            // 4. 打印 translation与rotation 数据
            RCLCPP_INFO(get_logger(),"平移：%f,%f,%f",translation.x,translation.y,translation.z);
            RCLCPP_INFO(get_logger(),"旋转:%f,%f,%f",r,p,y);
        }
        catch(const std::exception& e)
        {
            // 若 查询失败，则 执行 报错
            RCLCPP_WARN(get_logger(),"%s",e.what());
        }
        
        
    }
};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<TFListener>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}


