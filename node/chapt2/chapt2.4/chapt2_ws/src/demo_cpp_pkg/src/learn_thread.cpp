#include<iostream>
#include<string>
#include<thread>    // 多线程的头文件
#include<chrono>    // 时间 相关的 头文件
#include<functional>    // 函数包装器头文件
#include<algorithm>
#include"cpp-httplib/httplib.h" // 下载相关的头文件

class Download
{
private:
    /* data */
public:
    /*  下载 函数 ：通过 域名和地址，下载内容 */
    // 形参：第一个是 域名，第二个是 域名路径，第三个是 回调函数（类型 为 函数包装器;形参 为 两个 string的引用）
    void download(const std::string &host,const std::string &path,const std::function<void(const std::string&,const std::string &)>&callback_word_count){

        // 打印 线程的编号: std::this_thread::get_id()
        std::cout << "线程编号" << std::this_thread::get_id() << std::endl;                  // this_thread 表示 当前线程

        // 创建 客户端 对象
        httplib::Client client(host);

        // 获取 客户端的结果
        auto response = client.Get(path);

        // 判断 响应结果 是否不为空，其状态 是否等于 200
        if(response && response -> status ==200 ){                  // http 请求的 响应码 如果为 200 ，则 成功响应
            
            // 成功 响应，则 调用 回调函数
            callback_word_count(path,response->body);               // 第二个 形参 是 响应内容。（response为指针，内容 为 -> body）
        }
    };

    /*  启动下载函数 ： 目的是 主要用于 创建线程*/
    void start_download(const std::string &host,const std::string &path,const std::function<void(const std::string&,const std::string&)>&callback_word_count){

        // 对 download 函数 进行包装
        auto download_fun = std::bind(&Download::download,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

        // 创建线程 对象
        // 形参： 1. 目标函数。 2-n. 目标函数的形参 列表
        std::thread thread(download_fun,host,path,callback_word_count);              // 目标函数 为 回调函数的包装

        // C++ 创建线程 会 立马运行(一直运行，不会退出)，但是会 堵塞 当前线程。
        // 所以 需要将 当前线程 分离出来
        thread.detach();            // 分离出来，会 单独的在另外一个 线程中 运行
    }

};


int main(){

    // 创建一个class对象
    auto d = Download();

    /* 创建 回调函数： 字数统计作用 */
    auto word_count = [](const std::string &path,const std::string &result) -> void{

        // 字符串.length() 获取 字符串的长度（长度以char类型为单位）
        // 字符串.substr(n,m)   截取 字符串中 n到m的 内容 .  
        // 因为 UTF-8编码 是 3个字节 表示一个汉字，所以 尽量为 3的倍数
        std::cout << "下载完成" << path << result.length() << "->" << result.substr(0,9) << std::endl;  // 可以打印 3个汉字

    };

    // 开始下载: 启动 多线程
    d.start_download("http://0.0.0.0:8000","/novel1.txt",word_count);            // 域名 和 地址 要分开。由于用 函数包装器，所以 直接使用 函数名 即可
    d.start_download("http://0.0.0.0:8000","/novel2.txt",word_count); 
    d.start_download("http://0.0.0.0:8000","/novel3.txt",word_count); 

    // 将 主线程 休眠。防止 运行到 return 0，直接杀死 线程
    std::this_thread::sleep_for(std::chrono::milliseconds(1000*10));        // std::chrono::milliseconds(t) 计时 t 毫秒
    return 0;
}