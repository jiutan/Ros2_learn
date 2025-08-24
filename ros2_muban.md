# ROS2 学习过程中 遇到的模板
# 以后 可以 直接 复制粘贴 使用
## C++
### 一、C++程序
#### 1. 头文件：
```cpp
// 时间 库
#include "chrono"
// 命名空间：在 代码中 直接可以使用 数字s/ms 表示 几秒/毫秒
using namespace std::chrono_literals;
```
#### 2. 创建 类：
```cpp
class 类名:public rclcpp::Node		// 继承 Node类
{
private:		// 在 私有成员内部 声明 智能指针 对象 作为 类成员
	// 声明 模板类 智能指针对象
	std::shared_ptr<模板类> 类成员对象_;
	
	// 声明 定时器 对象
    rclcpp::TimerBase::SharedPtr timer_; 
public:			
	// 编写构造函数，并且给 该节点 起个名字
	类名():Node("节点名字")
	{
		// 在构造函数中 实例化 声明的对象
		this -> 类成员对象_ = std::make_shared<类模板>(参数)
		
		// 运行 功能/成员函数方法
		this -> 成员函数();
		
		// 使用 定时器：一段时间，调用 回调函数【使用 函数包装器】
		timer_ = this -> create_wall_timer(几ms,std::bind(&类名::回调函数,this));
	}
	
	// 定义 成员函数方法：实现 功能
	void 成员函数(){
		
	}
	
	// 定义 回调函数
	void 回调函数(){
	
	}

};
```
#### 3. 主函数：
```cpp
int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<上面定义的类>();

    rclcpp::spin(node);

    rclcpp::shutdown();
	
	return 0;
}
```

### 二、CMakeLists
```txt
''' 在 find_package 下面 '''
# 添加 可执行文件
add_executable(可执行文件名 src/节点文件名.cpp)

# 给 可执行文件 添加 依赖项
ament_target_dependencies(可执行文件名 依赖项)

'''	在 ament_package() 上面'''
# 安装 路径
install(TARGETS 可执行文件名
DESTINATION lib/${PROJECT_NAME}
)
```

### 三、 参数 服务
#### 1. 更新 其他节点 参数K 的 模板
```c
void update_server_param_k(double k){
    // 1. 创建 一个 参数服务 的对象（包含：  name 与 value 两个 参数）
    auto param = rcl_interfaces::msg::Parameter();
    // 2. 给 服务对象 name 赋值
    //      string name 与  ParameterValue value（复合消息接口）
    param.name = "k";
    // 3. 创建 value参数对象 ，其为ParameterValue消息接口 类型 
    auto param_value = rcl_interfaces::msg::ParameterValue(); 
    //  分别给 ParameterValue对象 的 参数 赋值
    // 优点：使用 消息接口 标准化
    param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;    // 类型为 double消息接口 类型 
    // 由于 是 double类型，可 直接赋值 double_value
    param_value.double_value = k;
    // 4. 将 服务对象 的 value 赋值 param_value
    param.value = param_value;
    // 5. 请求 更新/设置参数 并 将 返回的 结果 进行 处理
    auto response = this->call_set_parameter(param);
    // 判断 返回结果
    if(response == NULL)        // response 为 指针
    {
        RCLCPP_INFO(this->get_logger(),"参数 更新 失败！");
        return;
    }
    // 将返回的结果 进行 遍历处理
    // 其 response内容为 ：SetParametersResult[]数组，其 对象为 results
    //      内部参数为：bool successful 与  string reason
    for(auto result : response -> results){
        if(result.successful == false){
            // successful   是否成功
            // reason       失败的原因（注意使用：c_str() 转换为 C语言下的 字符串类型）
            RCLCPP_INFO(this->get_logger(),"参数 更新 失败！其原因为:%s",result.reason.c_str()); 
        }else{
             RCLCPP_INFO(this->get_logger(),"参数 更新 成功！"); 
        }
    }
}
```
## Python
### 1. 节点名.py 文件
```py
class 节点类名(Node):					# 继承 Node节点类
	# 构造函数 // 声明 定时器 对象
    rclcpp::TimerBase::SharedPtr timer_; 
	def __init__(self):
		# 1. 给 节点 起名字
		super().__init__('节点名字')	
		# 2. 创建 需要用到的 类成员 对象
		self.起个对象名字_ = 用到的类(参数)
	
	# 创建 成员函数（在 成员函数中，实现 对应 功能）
	def 成员函数名(self):
		'''
		实现 具体功能
		'''
		pass

def main():
	# 1. ros初始化
	rclpy.init()
	# 2. 创建 节点类
	node = 节点类名()
	# 3. 运行 节点
    rclpy.spin(node)
    # 4. 关闭 节点
    rclpy.shutdown()
```
### 2. setup.py 文件 环境配置
```py
	entry_points={
        'console_scripts': [
        	'可执行文件名字 = 功能包名.节点文件名:main'
        ],
```