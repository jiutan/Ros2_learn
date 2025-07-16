#include<iostream>
#include<functional>        // 函数包装器 的头文件

// 自由函数
void save_with_freefun(const std::string &file_name){

    std::cout << "自由函数" << file_name << std::endl; 

}

// 类：成员函数
class FileSave
{
private:
    /* data */
public:
    FileSave(/* args */)=default;               // 构造函数 : =default 默认为空
    ~FileSave()=default;                        // 析构函数 : = default 默认为空

    void save_with_class(const std::string &file_name){

        std::cout << "成员函数" << file_name << std::endl; 

    };
    
};

// 主函数
int main(){

    // 创建 类对象
    FileSave file_save;

    // 创建 Lambda函数 对象
    auto save_with_lambda = [](const std::string &file_name) -> void {
        std::cout << "Lambda函数" << file_name << std::endl; 
    }; 

    // 调用 自由函数
    save_with_freefun("file.txt");
    // 调用 成员函数
    file_save.save_with_class("file.txt");
    // 调用 Lambda函数
    save_with_lambda("file.txt");

    /*  使用 函数包装器 将 上面三种函数 统一成 一种 */
    // <> 中 方的是 模板：void为返回，(const std::string&)为 形参的类型
    std::function<void(const std::string& )> save1 = save_with_freefun;     // 将 自由函数 包装成 save1   
    std::function<void(const std::string& )> save2 = save_with_lambda;      // 将 lambda函数 包装成 save2
    // 将 成员函数 放入 包装器:【需要 绑定bind】
    std::function<void(const std::string& )> save3 = std::bind(&FileSave::save_with_class,&file_save,std::placeholders::_1);

    // 调用 包装函数
    save1("file.txt");
    save2("file.txt");
    save3("file.txt");

    


    return 0;
}


