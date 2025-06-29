// #include"rclcpp/rclcpp.hpp"
#include<iostream>
using namespace std;

/**
 * 函数：程序入口参数
 * 参数：
 *      argc表示 参数数量.
 *      argv表示 参数数组:                  （双重指针** 表示 二维数组）
 *                      第一个值argv[0] 表示 程序名字；[./a.out]
 * 示例：
 *     若执行 ./a.out ：则 argc = 1; argv[0] = ./a.out
 *     若执行 ./a.out --help: 则 argc = 2; argv[0] = ./a.out ; argv[1]= --help。（--help也为参数）
 *  */        
int main(int argc,char** argv){            

    cout << "参数数量= " << argc << endl;
    cout << "程序名字= " << argv[0] << endl;

    return 0;
}