#include<iostream>
#include<memory>

int main(int argc,char **argv){

    auto p1 = std::make_shared<std::string>("this is a str");  // std::make_shared<数据类型/类>(参数)          创建这个类所需的参数；返回值是一个 共享指针

    // 使用 .use_count() 来 得到 指向内存地址的 引用次数
    std::cout << "p1 的 引用计数" << p1.use_count() << std::endl;       // 为 1

    // 使用 .get() 来 获取 共享指针的地址(内存地址 为 p1指向的地址)
    std::cout << ",指针指向的内存地址为" << p1.get() << std::endl; 


    auto p2 = p1;           // p2 也 指向 p1的内存地址。此时，p1的地址 被 引用了两次

    std::cout << "p1 的 引用计数" << p1.use_count() << ",指针指向的内存地址为" << p1.get() << std::endl;        // 为 2
    std::cout << "p2 的 引用计数" << p2.use_count() << ",指针指向的内存地址为" << p2.get() << std::endl;       // 为 2 （指向同一个地址）

    // 释放引用/内存
    p1.reset();         // p1 被释放了.就 不指向 "this is a str" 所在的内存了
    std::cout << "p1 的 引用计数" << p1.use_count() << ",指针指向的内存地址为" << p1.get() << std::endl; // 为 0 （因为 p1 不指向 该地址了）
    std::cout << "p2 的 引用计数" << p2.use_count() << ",指针指向的内存地址为" << p2.get() << std::endl; // 为 1 （2-1）

    std::cout << "p2 指向的内存地址 数据：" << p2 -> c_str() << std::endl;       // p2 指针 调用成员方法c_str()将内存地址数据 用字符串打印出来
    // -> 为 指针获取内容
    
    return 0;

}