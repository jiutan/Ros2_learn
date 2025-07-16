#include<iostream>
#include<algorithm>

int main(){

    auto add = [](int a,int b) -> int {return a+b; };             // 相当于 一个函数
    
    int sum = add(200,50);
    
    // 定义一个print函数，让其来 打印
    // 将 sum对象 拿到 {}的作用域中
    auto print_sum = [sum]() -> int {

        std::cout << sum << std::endl;                          // 捕获到的 sum变量

    };

    // 使用 函数对象 进行打印
    print_sum();

    return 0;
}