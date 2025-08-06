// #include<rclcpp/rclcpp.hpp>
// 引入 QT 相关的 头文件
#include<QApplication>      // QT 
#include<QLabel>            // QT 的 标签
#include<QString>           // QT 里的 字符串类型

int main(int argc,char **argv){

    // 1. 创建 QApplication对象
    QApplication app(argc,argv);                // rclcpp很像

    // 2. 创建 QT 标签 的 指针对象(在 堆区)
    QLabel* label = new QLabel();

    // 3. 创建 QT 字符串类 的 对象 名字叫 message
    QString message = QString::fromStdString("Hello Qt!");           // fromStdString方法：来自于 C++版本 的 标准字符串

    // 4. 将 message对象 放入 label类对象 中
    label -> setText(message);

    // 5. 准备显示
    label -> show();

    // 6. 执行 应用：显示 
    app.exec();             // 会 阻塞代码

    return 0;
}