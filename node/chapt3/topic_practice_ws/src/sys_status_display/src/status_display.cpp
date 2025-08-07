// qt5 相关库
#include<QApplication>
#include<QLabel>
#include<QString>
// ros2 相关库
#include<rclcpp/rclcpp.hpp>
// 自定义消息接口 库 的 文件
#include<status_interfaces/msg/system_status.hpp>

// 定义 命名空间
using SystemStatus = status_interfaces::msg::SystemStatus;          // 用 SystemStatus 代替 后面 的 命名区域

class SysStatusDisplay:public rclcpp::Node                          // 继承 节点类
{
private:
    // 使用 共享指针 方法 来 创建 订阅者对象 来 订阅话题
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;

    // 声明 qt标签的 对象
    QLabel* label_;

public:
    /*      继承的要求：需要在 子类的构造函数中 调用 父类的方法    子类():父类()     */
    SysStatusDisplay():Node("sys_status_display"){

        // 初始化 qt标签 对象
        label_ = new QLabel();

        // 初始化 定义 订阅者对象（话题名字 需与 发布者 一致）
        /* 【技巧】内部创建回调函数 的 方法：[&] 捕获 上下文 中所有的列表；()内为 形参列表,里面是 消息接口的共享指针;       */
        subscriber_ = this -> create_subscription<SystemStatus>("Sys_Status",10,[&](const SystemStatus::SharedPtr msg)->void{           

            // 订阅者拿到 数据后，要在 回调函数 中 显示 出来
            label_ -> setText(msg_to_qstr(msg));                // 将 msg消息 组装成 QString类型 给到 label_中 进行显示


        });

        // 构造函数运行完，显示 默认内容(无 数据的 内容)。      当 没有消息时，只显示 项目      
        label_ -> setText(msg_to_qstr(
            std::make_shared<SystemStatus>()                        // 创建 空的 智能指针. 
        ));         
        label_ -> show();

    };

    // 创建函数： 将 收到的消息内容msg，转化/拆成 QString类型
    QString msg_to_qstr(const SystemStatus::SharedPtr msg){

        /*          字符串 组装 方法          */ 
        // 1. 创建 stringstream 的 show_str 对象
        std::stringstream show_str;
        // 2. 使用 show_str 方法
        show_str << "========系统状态可视化工具==============\n"
        << "数 据 时 间： \t"  << msg -> stamp.sec << "\ts \n"                  // 单位：   s
        << "主 机 名 字： \t"  << msg -> host_name << "\t \n"                   // 无单位
        << "CPU 使 用 率： \t"  << msg -> cpu_percent << "\t% \n"               // 单位：   %
        << "内 存 使 用 率： \t"  << msg -> memory_percent << "\t% \n"           // 单位：   %
        << "内 存 总 大 小： \t"  << msg -> memory_total << "\tMB \n"            // 单位：   %
        << "剩 余 内 存 大 小： \t"  << msg -> memory_available << "\tMB \n"      // 单位：   %
        << "网 络 数 据 发 送 量： \t"  << msg -> net_sent << "\tMB \n"           // 单位：   %
        << "网 络 数 据 接 收 量： \t"  << msg -> net_recv << "\tMB \n"           // 单位：   %
        << "================================================"
        ;

        return QString::fromStdString(show_str.str());
    }


};

int main(int argc,char *argv[]){

    rclcpp::init(argc,argv);

    QApplication app(argc,argv);

    auto node = std::make_shared<SysStatusDisplay>();               // 创建 类的 共享指针 作为 结点

    /*      需要 使用 多线程 ： 否则 一个阻塞代码，另一个就执行不了    */
    // 创建 一个线程thread
    std::thread spin_thread([&]()->void{
        // 单独开一个线程 执行 rclcpp::spin()。     这样 不会阻塞代码，导致 无法运行 app.exec()
        rclcpp::spin(node);                     // 执行应用，阻塞代码（需用 多线程 分别执行）

    });
    spin_thread.detach();                   // 退出 线程

    app.exec();                             // 执行应用，阻塞代码（需用 多线程 分别执行）

    return 0;
}

