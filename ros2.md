# ROS2学习
## 一、ROS2安装与rosdep环境的配置
### 1.使用小鱼 ROS2 一件安装
1. 下载脚本并执行脚本（脚本运行完会自动删除）
```c
    wget http://fishros.com/install -O fishros && . fishros
```
2. 选择ROS相关： 1 ROS安装
3. 选择选择是否换源： 1 更换系统源再继续安装
4. 选择是否清理第三方源： 2 更换系统源并清理第三方源
5. 选择ROS版本： 1 humble版本
6. 选择具体版本： 1 桌面版
7. 若显示加入学习交流群，则说明安装成功
### 2.配置rosdep
1. 下载脚本并执行脚本（脚本运行完会自动删除）
```c
    wget http://fishros.com/install -O fishros && . fishros
```
2. 选择ROS相关： 3 配置rosdep
3. 等待安装
4. 检验配置是否成功：在终端输入`rosdepc update`


## 二、环境变量的妙用
### 1.配置环境变量
注意：每次**打开新的工程 或 编译工程时**，都要进行环境变量的配置与检查
- 环境配置文件：`.bashrc`文件(在home文件夹下，被隐藏了)
- 当更改环境配置文件后，要*重新打开终端*。（因为可能该终端已经运行过该文件了）

### 2.想修改打印出来的 基础提示内容（如：行号，时间辍等）
1. 在shell命令中，输入：
```c
	// 将 输出内容的前缀 改为： <运行命令的 行数>:<输出内容>
	export RCUTILS_CONSOLE_OUTPUT_FORMAT=[{function_name}:{line_number}]:{message}
```
### 3. C++ rclcpp库 的 路径配置
`/opt/ros/humble/include/**`

## 三、Shell命令代码
### 1.输出打印指令【echo】【$的用途】
#### echo:输出与打印 内容
```c

    echo Hello_World		// 打印 Hello_World

```
#### $+<程序名>：终端会找到 $ 之后的字符 对应的环境变量后，用该环境变量替换 $ 字符
```c
    
    echo $ROS_VERSION		// 查看ROS的版本：ROS1 OR ROS2
    
    echo $ROS_DISTRO		// 查看ROS2当前程序的版本

    printenv                // 查看Linux下 所有的环境列表

```
#### printenv 与 grep: 打印查找的内容
```c
    printenv | grep <需要查找的内容部分>		// 打印出 含查找部分的 内容
```
### 2.找出文件【export】
```c

    export <文件名>

```

### 3.ROS2 命令
#### run：运行命令[通过环境变量查找文件]
```c
    ros2 run <功能包的名字> <可执行文件的名字>      // 本质：就是 运行可执行文件
```
注意：ros2 run 是通过**环境变量（AMENT_PREFIX_PATH=/opt/ros/humble）来查找功能包以及可执行文件的**（不是在当前目录）
查找文件步骤：
（1）通过环境变量：AMENT_PREFIX_PATH
（2）查找：lib/package_name(功能包名)/exacuteable_name(可执行文件名)
#### source ：添加 环境变量

#### node ：查看 节点相关信息
```c
    ros2 node list		// 查看当前 正在运行 的节点列表

    ros2 node info <正在运行的节点路径>     // 查看指定节点的详细信息
```


## 四、节点node学习
### 1.基础内容
1. **python**的node初始化
```python
import rclpy			    # 引入 rclpy库
from rclpy.node import Node         # 从 rclpy库中的node类 导入到 Node中

# 定义 main函数：ROS2节点的基础步骤（python）
def main():
    rclpy.init()                    # 1.初始化工作，分配资源
    
    node = Node('pyhton_node')      # 2.创建节点的实例对象（创建一个节点名字python_node，将 Node类的 python_node这个实例（对象） 赋值给了 node变量）
    
    # 在这里 插入功能语句
    
    rclpy.spin(node)                # 3.循环的运行检测节点，直到节点关闭为止（只要不打断，不会打断）
    
    rclpy.shutdown()                # 4.关闭并清理节点（推出后，需清理节点资源）

# 定义模块：如果该模块是主模块（__main__），则运行 main()函数. 用于判断当前脚本是直接运行还是作为模块被导入到其他脚本中执行
if __name__=='__main__':            
    main()
```
2. **C++文件**的node初始化
  - C++**源程序**
```c
#include"rclcpp/rclcpp.hpp"             // 引入头文件.hpp【需要导入库，使用CMakeList查找并导入】

int main(int argc,char** argv){         // 固定写法，使用args：具体看 learn_args.cpp           

    // 1. ros初始化
    rclcpp::init(argc,argv);            // 其中，argc和argv可以传递给rclcpp中，可用于修改其它东西。
    
    // 2. 创建node节点,名字为：cpp_node
    auto node = std::make_shared<rclcpp::Node>("cpp_node");   // 创建rclcpp下的node的共享指针。（make_share为共享指针）

    // 自定义功能

    // 3. 运行节点，循环直到中断
    rclcpp::spin(node);                 // spin是不断循环检测，并处理node事件

    // 4. 关闭节点，清理资源
    rclcpp::shutdown();

    return 0;
}
```
  - C++**编译：CMakeList**(需要导入头文件与库文件)
```c
// cmake基础
cmake_minimum_required(VERSION 3.8)

project(ros2_cpp)

add_executable(ros2_cpp_node ros2_cpp_node.cpp)     # 可执行文件 与 c++文件

// 难点，重点：查找头文件包
find_package(rclcpp REQUIRED)           # 必须有，才能打印
message(STATUS ${rclcpp_INCLUDE_DIRS})  # 打印 rclcpp的头文件路径及rclcpp依赖的头文件
message(STATUS ${rclcpp_LIBRARIES})     # 打印 rclcpp的库文件及rclcpp以来的库文件

// 难点，重点：将头文件和库文件，与c++文件连接
target_include_directories(ros2_cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS}) # 头文件包含

target_link_libraries(ros2_cpp_node ${rclcpp_LIBRARIES})    # 库文件连接
```

### 2.日志打印【 node.get_logger() 】
#### 参数

#### 程序
- python程序：(在 `rclpy.spin(node)`前执行)
```py
    
    node.get_logger().info('Hello World!')  # 获取并使用get_logger日志管理模块，打印info级别的提示

```
- c++程序：(在 `rclcpp::spin(node)`前执行)
```c
    RCLCPP_INFO(node->get_logger(),"hello world!")  // RCLCPP_INFO 为一个 宏定义; 使用node指针 找到get_logger函数
```
#### 输出
输出内容：`[级别] [时间辍] [节点名字] : [打印内容]`
  - 级别：（1）info：普通级别；（2）warn：警告级别
  - 时间辍：从1970年至今的 秒数

## 五、功能包 pkg   【组织节点的工具】（在 功能包 中编写节点）
### Python语法：
1. 创建功能包 package：
```py
# 其中： --build-type ament_python ：表示 使用的构建类型 为 ament_python
#       --license Apache-2.0     : 表示 证书 为 Apache-2.0 
ros2 pkg create <功能包名> --build-type ament_python --license Apache-2.0
```
后生成功能包文件夹，包含：
  - 文件夹 功能包名：存放 **节点代码** 的文件夹。
  - 文件夹 resource：
  - 文件夹 test：存放 **测试代码** 的文件夹。
  - LICENSE：功能包的许可证.
  - package.xml	： 功能包 清单文件。用于声明 依赖库
  - setup.cfg：存放 python包的 配置选项。
  - setup.py：构建脚本文件。
2. 在里层`功能包名`文件夹中：新建 python_node.py 并 填写代码
3. 节点的注册与声明：在`setup.py`文件中，对main函数进行声明
```py
	# 相当于 main函数 与 可执行文件 进行映射
	entry_points={
		'console_scripts':[
			# 在 此处进行声明
			'<可执行文件名(自己取)> = <功能包名字>.<节点的文件名>:main'
		]
	}
```
4. 依赖库的声明：在`package.xml`中，添加需要的 依赖库信息。如：
```py
    # 在 </license> 后，添加
	<depend> 依赖库名 </depend>		# 如：<depend>rclpy</depend>
```
5. 功能包 **构建 colcon build**：
```py
	# 构建 功能包
	colcon build
```
- 构建后，会 在上层目录 生成三个文件夹：
  - 文件夹 build： 存放 构建过程中 产生的 中间文件。
  - 文件夹 install：存放 **构建结果**的文件夹。
  构建后，生成的可执行文件，在：
	`install -> 功能包名 -> lib -> 功能包名 -> 可执行文件`
  - 文件夹 log：
- 注意：运行的代码 是 被拷贝在lib中的代码，不是之前功能包中的代码。
- 所以：更改代码后，==一定要重新构建==
6. 构建、编译后，一定要**生成 并 修改环境变量**：
```py
	# 生成 环境变量
	# 通过source 运行 install文件夹下 的 setup.bash文件
	# 该文件是 将环境添加至系统环境变量 的脚本
	source install/setup.bash
	
	# 修改 环境变量
	echo $AMENT_PREFIX_PATH
```
7. 运行 功能包下的节点
```c
	ros2 run <功能包名> <节点名>
```


#### c++语法：
1. 创建功能包 package：
```c
    // 其中： --build-type ament_cmake ：表示 使用的构建类型 为 ament_cmake
    //       --license Apache-2.0     : 表示 证书 为 Apache-2.0 
ros2 pkg create <功能包名> --build-type ament_cmake --license Apache-2.0
```
生成一个 功能包目录，包含：
  - 文件夹 include：存放 C++头文件
  - 文件夹 src：存放 节点及相关代码
  - CMakeLists.txt：c++编译配置文件
  - LICENSE：许可证
  - package.xml：功能包的清单文件，包含 依赖信息。
2. 在 `src`文件夹下 新建文件，在文件中 编写 节点代码。
3. 节点的声明 与 拷贝文件：**配置 CMakeLists.txt文件**：
在`find_package(ament_cmake REQUIRED)`后，添加：
```c
find_package(<依赖库名> REQUIRED)           // 查找库（紧贴着find_package）

add_executable(<可执行文件executable名> <src/节点.cpp >) // 添加并命名 可执行文件

ament_target_dependencies(<可执行文件名> <依赖库名>)	// 功能包中，独有

// install安装命令 
// 拷贝文件：将build目录下的executable文件 拷贝至 install目录下的lib中
// 其中，${PROJECT_NAME}会随 工程名变化
install( 
	TARGETS <executable file>	
	DESTINATION lib/${PROJECT_NAME}	
)
```
4. 依赖库的声明：在 功能包文件夹 下的**package.xml**中，添加依赖信息。
```c
	// 在</license>后，另起一行，添加
	<depend> 依赖库 </depend>			// 如：rclpy
```
5. 在上级目录下(chapt文件夹下)，**构建 colcon build**:
```c
	colcon build
```
6. 添加 环境变量
```c
	source install/setup.bash
```
7. 运行 功能包下的 节点程序文件
```c
	ros2 run <功能包名> <可执行文件名>
```

## 六、工作空间 Workspace
1. Workspace：一个完整的机器人项目，往往由多个不同的 功能模块组成，所以就需要 多个功能包进行组合。（workspace 只是一个 概念，其实 ==本质 就是一个**文件夹**==）

### 1. 创建 工作空间
```c
	// -p:表示 创建双重文件夹： 文件夹1/文件夹2(文件夹1 下的 文件夹2)
	mkdir -p <文件夹名>_ws/src
```
### 2. 构建 功能包
```c
	// 构建 工作空间中 所有的功能包
	colon build
	
	// 构建 工作空间下 单独的功能包
	colcon build --packages-select <pkg名字>
```
### 3. 调整 构建的先后顺序（功能包的依赖顺序）
若   想先构建 CPP文件，再构建 python文件
需  将cpp文件 ，放入 python功能包中的依赖途径`package.xml`中
```c
	// 在 python功能包中的 package.xml 文件中
	// 在 <buildtool_depend> 前 加入
	<depend>demo_cpp_pkg</depend>
```
## 七、话题topic 通信学习
### 1. 介绍：
#### 关键点
1. **发布者**：消息内容 的 创建者
2. **订阅者**：消息内容 的 接收者
3. **话题名称**：发布消息的内容 名称
4. **话题类型（消息接口）**：消息的类型 
#### 查看 话题、接口定义 的 方法【启动中的】
1. 查看 正在运行的 ==节点列表==：`ros2 node list`
2. 查看 ==节点 的具体信息== ：`ros2 node info /<功能包>`。
  - Subscribers 中：订阅者
    - 冒号：前的 为 **消息的名称**
    -  冒号：后的 为 **消息接口**
  - Publishers  ：发布者
3. 查看 正在运行的 ==话题列表--：**`ros2 topic list -t`**
4. 实时查看 ==话题 的内容== ：`ros2 topic echo <话题的名字>`（<>省略）
5. 查看 ==话题的 详细信息== ：`ros2 topic info <话题的名字> -v`
6. 查看 ==消息接口的 定义的详细内容==：`ros2 interface show <消息接口的名字>`
7. ==**发布 话题**==：`ros2 topic pub <话题的名字> <消息接口> "消息接口的内容填充"`
  - 消息内容填充：`"{vector: }"`				
    - 只要遇到 变量 就要加个{ }
    - 冒号: 后的 要添加 **空格**
```c
// 让 小海龟 动起来
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0} , angular: {z: -1.0}}"
```
### 2. Python 话题 的 发布与订阅
#### 【项目1】通过 话题 发布 小说 
1. 实现功能：下载 小说，并通过 话题 间隔5s 发布一行
2. 核心问题：
  1) 怎么下载 小说？		request
  2) 怎么发布？			 确定 话题名字 和 消息接口(string接口)
  3) 怎么间隔5s发布？	        TImer定时器
3. 搭建 网站 并 开启 服务器：`python3 -m http.server`
  - 会将 文件夹 下的内容，传至 服务器
##### 项目实现
第一步：创建 工作空间：
  - 使用的 依赖包：`rclpy`和`example_interfaces`
```c
// 在 ws/src 下 创建 工作空间
ros2 pkg create demo_python_topic --build-type ament_python --dependencies rclpy example_interfaces --license Apache-2.0
// 在 ws 下 构建
colcon build
```
##### 项目检查
1. 查看 正在发布的话题 列表：`ros2 topic list`
2. 查看 话题 的 内容：`ros2 topic echo 话题名字`
3. 查看 话题 发布 的 频率：`ros2 topic hz 话题名字`
#### 【项目2】订阅小说 并 诸行朗读
##### 核心问题：
1. 怎么订阅？
2. 用什么来 朗读文本？			**Espeak库**
3. 小说来的快，读的太慢 怎么办？	 **使用 队列 可以 异步处理**
##### 项目实现：
1. 下载 Espeak库：`pip3 install espeakng`与`sudo apt install espeak-ng`
2. 下载后，重启scode
3. 使用到的库：
  - Espeak库：`import espeakng`
    - 用于 读书
  - ros库：`import rclpy`,`from rclpy.node import Node`
    - 用于 继承 节点对象
  - 消息接口库：`from example_interfaces.msg import String`
    - 用于 确定 传递消息的 类型
  - 队列库：`from queue import Queue`
    - 用于 将小说存入队列中，再从队列中读出来，实现异步读取，防止错乱
  - 线程库：`import threading `
    - 用于 创建一个线程，用来 单独读小说。（ros2为 单线程）
  - 时间库：`import time `
    - 用于 让 不用的线程 休眠
4. 实例化 的 对象：【**在 节点类 中**】
  - 创建 队列 对象：`slef.对象 = Queue()`
  - 创建 订阅者 对象：`self.对象 = Node.create_subscription(parameter)`
    - 参数：
    - msg_type消息类型（消息接口）
    - topic话题： 需与 发布者 发布的 话题名字 一致，才能够 订阅的到
    - callback回调函数： 订阅的时候 需拿取数据。 需 通过回调函数 来 告诉系统 数据接收到了（类似 中断）
    - qos_profile：保持与 发布者 一致即可
  - 创建 多线程 对象：`self.线程对象 = threading.Thread(target=self.使用线程的函数名)`
  - 启动 多线程对象：`self.线程对象.start()`	# python的线程无法 自己启动，需 启动线程。并且 跳转到 speak_thread成员函数 中，运行
5. 定义 成员函数：【**在 节点类 内**】
  - 回调函数：**有消息时，则自动调用 该函数**
```py
	# 在 类 中 定义 回调函数：有消息时，则 会调用 该函数
    # 发布者 发布一个 msg；订阅者 就接收一个 相同内容的msg（ros2后台完成）
    def novel_callback(self,msg):           # msg 为 novel_pub_node.py中的 msg=String() 同一个
        # 将 小说 放入队列中（put）
        self.novel_queue_.put(msg.data)     # msg本身 为 string对象，我们只需要 msg中的data成员
```
  - 线程函数：**在 单独的线程，读书**
```py
 # 使用 线程 的 函数：用于 语音读书
    def speake_thread(self):
        # 1. 先 创建一个 speaker（实例化Speaker类）
        speaker = espeakng.Speaker() # 生成 一个 Speaker类的对象叫 speaker
        # 2. 设置 语音 的 语言
        speaker.voice = 'zh'
        # 3. 检测 当前上下文 是否正常ok
        while rclpy.ok():
            # 4. 若正常，则 判断 队列大小是否不为0（是否接收到消息）
            if self.novel_queue_.qsize() > 0:
                # 5. 若 队列中 有内容，则 从 队列中 取出 文字 存入 text中
                text = self.novel_queue_.get()
                # 6. 存入 文字后，打印出来 查看 是否收到
                self.get_logger().info(f'朗读：{text}')
                # 7. 调用 语音 进行 读 text中的 内容
                speaker.say(text)
                # 8. 等待 说完
                speaker.wait()      # 说完后，才可 读下一句
            else:   
                # 5. 关键点：若 队列中 没有内容， 则 需 让 当前的线程 休眠【使用 time库】（降低CPU功耗）
                time.sleep(1)     # 休眠 1s。（来 降CPU功耗 ）
```
##### 项目检查：
##### 项目拓展：
1. 拓展1：==优化声音==
- 原因：声音难听，需要优化
- 方案：接入**第三方API**
2. 拓展2：小说单调，换个小说

### 3. C++ 话题 的 发布与订阅
1. 在 机器人 的开发项目 中：C++主要用于 做 运动控制
#### 【项目一】控制 小海龟 画指定半径的圆
##### 分析问题：
1. 小海龟 凭什么 听我的？			通过**话题**
2. 如何 画圆？				      **$半径 = 线速度/角速度$**
3. 如何 循环发送？				  使用**定时器**
4. 小海龟 的 话题：
  - 控制命令(订阅)：	话题：/turtle1/cmd_vel		消息接口：geometry_msgs/msg/Twist
  - 当前坐标(发布)：	话题：/turtle1/pose		消息接口：turtlesim/msg/Pose
##### 项目实现
##### 项目检查

#### 【项目二】订阅 Pose 实现 闭环控制
需求：告诉 小海龟 到指定位置，自己滚过去
##### 分析问题：
1. 小海龟 凭什么听我的？			
  - 通过 **发布话题**
    - 话题名称：`/turtle1/cmd_vel` 
    - 话题类型/消息接口：`geometry_msgs/msg/Twist`
2. 小海龟 现在在哪里？			
  - 通过 **订阅话题**
    - 话题名称：`/turtle1/pose`
    - 话题类型/消息接口：`turtlesim/msg/Pose`
3. 怎么根据 当前位置和目标位置 计算 角速度和线速度？		
	答案： 两点之间距离 -> 线速度	； 当前朝向 和 目标朝向差 -> 角速度
#### 4. 话题通信 实践
##### 步骤一： 确认 需求
1. 通过 该小工具，可以看到 系统的 实时状态信息，包括 记录信息的时间、主机名称、CPU使用率、内存使用率、内存总大小、剩余内存、网络接收数据量 和 网络发送数据量
2. 要有一个 简单的界面，可以将 系统信息 显示出来
3. 要能在 局域网内 的 其他 主机上 查看数据
##### 步骤二：根据 需求 进行 分析
1. 要获取 系统状态信息：	使用 python库中的 psutils
2. 要 有一个 展示界面：	   使用 C++ 调用 Qt库
3. 要能 共享数据：		     使用 ROS2话题 功能
##### 步骤三：自定义 通信接口
1. 原因：每个话题，都要有其 对应的 话题类型/通信接口。
- 但是 ROS2中 接口是有限的 **没有满足符合所有需求的接口，需要 自定义 一个**
2. ==在 项目中，要 根据 项目需求，去创建**自定义 的 通信接口**来 通信==
###### 步骤：
第一步：方法（使用C++功能包）：
- 在工作空间 下 的 src文件夹下：使用`ros2 pkg create status_interfaces --dependencies builtin_interfaces rosidl_default_generators --license Apache-2.0`
  - 功能包：status_interfaces
  - 依赖（--dependencies）：
    - builtin_interfaces：时间辍 消息接口
    - rosidl_default_generators：将 自定义消息文件 转换成 C++/Python 源码的 模块

第二步： 在 status_interfaces 文件夹 下 名字为 **`msg`**文件夹【规定好的名字，不能改】

第三步：==在`msg`文件夹 下，创建**消息接口**文件==
  - 消息接口 文件名：首字母需大写
  - 消息接口 文件**后缀：`.msg`**
    - SystemStatus.msg：

第四步：对 消息接口 文件 进行 编写
- 使用 interface 功能包
- 若 该消息接口 依赖 interface功能包 中的 消息接口，则：**省略 中间的 msg**【规定】
- 在 该文件 中，定义 接下来 需要使用的 消息接口

第五步：将 该文件 在 **`CMakeLists.txt`**文件中 配置（在 `if(BUILD_TESTING)`前）
- 添加 需要 的 依赖功能包：`find_package(rosidl_default_generators REQUIRED)`
- 使用 用于 产生 功能包 的 指令：**`rosidl_generate_interfaces(参数)`**
  - 该 cmake函数 指令 的 作用：** 将 msg等 消息接口定义文件，转换成 库或者头文件类**
  - 参数：【规定：参数之间 用 ==回车== 隔开】
    - 1. 工程名字：                    ${PROJECT_NAME} 
    - 2. msg消息接口文件 的 相对路径：  "msg/消息接口文件.msg"  (若有多个 ，在 后面写即可，用 回车 隔开即可)
    - 3. 自定义 新的消息接口 需要的依赖： DEPENDENCIES 需要的依赖
```CMakeLists 
rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/SystemStatus.msg"
  DEPENDENCIES builtin_interfaces
)
```

第六步：在 `package.xml`文件中：（在`/license`后 添加）
- 添加 命令：**`<member_of_group>rosidl_interface_packages</member_of_group>`**
  - 作用：告诉大家，该== 功能包 是 **包含 消息接口** 的 功能包==

第七步：`colcon build`编译（在 工作空间ws 编译）
- 编译完后，会在`_ws/install/status_interfaces/include/msg`路径下
  ==生成 C++ 所需 的 **`消息接口文件名.hpp`**文件==
  - 其中，生成的 类 在 `detail/消息接口__struct.hpp`文件下 声明
- 会在 `status_interfaces/local/lib/python3.10/status_interfaces/msg`路径下
==生成 python 文件 __init__.py，里面有 导入库的语句：（需 与 文件内的 一致）
**`from status_interfaces.msg._system_status import SystemStatus`**

第八步：使用 生成的 hpp 文件 和 python库 进行 下一步操作

###### 查看 消息接口 具体内容
可以使用 `ros2 interface show 功能包名/msg/消息接口文件` 来查看
##### 步骤四：使用 自定义消息接口 
1. 在 创建功能包 时：要 加入 **消息接口依赖**，`--dependencies 消息接口功能包名/`
2. 在代码中：
导入 自定义的消息接口：**`from 消息接口功能包名.msg import 消息接口文件名`**

###### 系统消息 的 获取与 发布
1) 在src中：构建 python功能包
`ros2 pkg create py_status_publisher --build-type  ament_python --dependencies rclpy status_interfaces/ --license Apache-2.0`
2) 

##### 步骤五： 订阅 数据 并用 QT 显示
1. Qt：跨平台的软件设计与 开发工具
2. 作用：可以用于 **界面  显示 的 设计**
###### 在 功能包中 使用 Qt（学习 完成 界面显示 功能）
1. 创建 Qt功能包 ：
  - 构建类型：`--build-type ament_cmake`
  - 依赖：`--dependencies rclcpp 自定义消息接口功能包`
  - 许可证：`--license Apache-2.0`

2. Qt 相关的 头文件：
```c
#include<QApplication>
#include<QLabel>
#include<QString>
```
3. Qt 相关的 类 与 实现：
```c
int main(int argc,char *argv[]){
	QApplication app(argc,argv);
	QLabel* label = new QLabel();
	QString message = QString::fromStdString("Hello Qt!");
	label -> setText(message);
	label -> show();
	app.exec();
	return 0;
}
```
4. CMakeLists.txt 填写：
  1. **查找库** Qt5 图形界面组件：`find_package(Qt5 REQUIRED COMPONENTS Widgets)`

    - COMPONENTS：说明 添加的组件
    - Widgests：Qt 图形界面 组件
  2.**生成可执行文件**
添加 可执行文件：`add_executable(可执行文件名 src/待生成文件的路径)`
  3. **链接库**
给 可执行文件 添加 Qt5 的 依赖：`target_link_libraries(可执行文件 Qt5::Widgets)`

    - target_link_libraries：因为 Qt5 不是 ros库中的，所以使用 该命令
  4. **拷贝** ：在 endif 下 加上 install 参数
```c
install(TARGETS 可执行文件名
DESTINATION lib/${PROJECT_NAME}
)
```
###### 订阅数据 并用 Qt 显示
1. 在 vscode .json 文件中 添加路径配置：
  - 查询 qt5文件 路径位置 代码：
在 终端 输入**`whereis qt5`**查询，将`lib`替换成`include`
```json
"includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "/usr/include/x86_64-linux-gnu/qt5/**"
            ],
```


## 八、服务通信 与 参数通信
1. ROS2 的 四种 通信方式：
  - ==话题通信==【基础】：**单向通信**，发布者 发给 订阅者
  - ==服务通信==（两个话题通信构成）：**双向通信**，有接有发
  - ==参数通信==（多个服务通信构成）：用于 **修改参数**
  - ==动作通信==： 
### 1. 服务通信（service）
1. 三要素： 
- 服务 通信名字
- 服务端 与 客户端
- 服务 消息接口
#### 服务通信 的 ROS2 命令
1. 查看 正在运行的ros2 下的 服务模块 以及 接口类型：`ros2 service list -t`
  - 服务名字：/ + 名字
  - 服务的 消息接口/服务类型： [ 里面的 内容]
2. 查看 具体的 服务模块：`ros2 interface show 服务消息接口`
  - “---” 上面部分：是 **请求部分**。要使用该接口 时 ，需创建 的 请求内容。
  - “---” 下面部分：是 **返回部分**。使用 该接口后，该接口 将 返回 的值。
3. 服务通信 的 请求（终端命令行）：
- 语法：`ros2 service call 服务名字 服务消息接口 "{请求部分的数据(用逗号隔开)}"`
如：`ros2 service call /spawn turtlesim/srv/Spawn "{x:1,y:1}"`
4. 服务通信 的 请求==（rqt可视化界面）==：
- 语法：**`rqt`**，会生成 一个界面。
- 选择 `Plugins -> Services -> Service Caller`
- 可以在 Request区，更改信息 -> 点击 Call 发送
### 2. 参数通信（shell 命令）【基于 服务的】
1. ==**帮助** 查看 参数==：`ros2 param --help` 

2. ==参数==：被视为 **结点的设置**，是 *基于 服务通信* 实现的

3. 查看 当前运行程序 与参数 相关的 服务列表：
语法：`ros2 service list -t | grep parameter`

4. 使用【shell命令】 来 设置 参数：（设置 单个 参数）
- 查看 ==参数 列表==：`ros2 param list`
- 查看 ==某参数 的具体 内容==-：`ros2 param describe /消息名称 参数名称`
  - name：参数名称
  - Type：参数类型
  - Description：参数 描述
  - Constraints：参数 约束。（step：补偿）  
- ==**获取** 当前 参数的值==：`ros2 param get /节点名字 参数名称`
- ==**修改** 目标 参数 的 值==：`ros2 param set /节点名字 参数名称 设置参数值`
  - 返回：`set parameter successful` 

5. 使用【配置文件】来 设置 参数：（设置 多个 参数）
- 将 ==参数 **导出** 成 yaml文件== ：`ros2 param dump /节点名字 > 参数文件.yaml`
- ==**查看  yaml 文件**==：`cat 参数文件.yaml`
- == 修改 yaml文件== ：用于 改变 节点的 参数值
- ==**使用** yaml文件 来 修改参数==：
`ros2 run 功能包 节点名字 --ros-args --params-file 参数文件.yaml`

6. 使用【 rqt 可视化界面】来 设置 参数：
1) 打开 可视化界面：`rqt`
2) 打开 `Plugins -> Configuration -> Dynamic Reconfigure` 打开 参数 列表
3) 使用  该 参数界面 来 **动态的** 调整参数
### 3. 【Python服务通信项目】 实现 人脸检测
#### （1）自定义 服务接口
1. 目标：使用python创建客户端与服务端，完成 请求和响应。使用 视觉识别，实现 人脸检测 服务。
2. 大致内容：
- 创建 服务消息接口 与 服务端，用于 接收图片 并 进行识别
- 创建 客户端 节点，请求 服务，并 显示 识别结果
3. 需求：创建 人脸检测 服务，提供图像，返回 人脸 数量位置 信息
4. 难点分析：
  - 人脸怎么识别？			 	       使用`face_recognition库`
  - 图片数据和结果 怎么 传递？		 使用 服务通信
  - 没有合适的 消息接口 怎么办？ 	自定义服务消息接口（请求部分 为 图像；返回的是 位置信息）

5. 创建 自定义消息接口功能包 时 ==需要的 依赖==：
  - ==**创建自定义消息接口 必备**的依赖：**`rosidl_default_generator`**== 
  - ros2 图像消息接口 依赖：`sensor_msgs`
6. 删除 include 和 src，并在 功能包 下：
- 添加 **`srv` 服务接口文件夹**，并在下面 创建 **服务接口文件`文件名.srv`**
7. ==如何 自定义服务接口==：【	中间用 三个"-"隔开：---	】
- **请求**部分：使用 ros2中的 **图像定义 消息接口 `sensor_msgs/Image`**
- **响应/返回**部分：
```py
# 请求 部分
sensor_msgs/Image image		# 图像消息，原始图像
---
# 返回 部分
int16 number 		# 人脸个数
float32 use_time	# 识别 消耗的时间
int32[] top			# 使用数组（可能不止一个图像），来存储 人脸在图像中的位置
int32[] right
int32[] bottom
int32[] left
```
8. 在 CMakeLists.txt 中 添加 消息接口：
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FaceDetector.srv"		# 消息接口 的 路径
  DEPENDENCIES sensor_msgs		# 消息接口功能 的 依赖文件
)
```
9. 修改 功能包 消息文件 `package.xml`：**声明 该功能包 是 消息接口功能包**
在<depend>前，输入：
```xml
 <member_of_group> rosidl_interface_packages </member_of_group>
```
10. 构建 工作空间：`colcon build`；后，添加 环境：`source install/setup.bash`
11. 查看 定义好的 自定义消息接口：`ros2 interface show 功能包名/srv/文件名`
#### （2）人脸检测 的 实现
1. 人脸检测 库的 安装：`pip3 install face_recognition -i https://pypi.tuna.tsinghua.edu.cn/simple`
2. 创建 python功能包
3. 下载图片 并 ==在`setup.py`中 增加 新增的图片 地址：
```py
data_files=[
       ('share/' + package_name+'/resource', ['resource/图片路径']),
// 构建时，将图片拷贝到：/install/package_name/share/package_name/resource
    ],
```
4.  写 程序：`learn_face_detect.py`
#### （3）人脸检测 服务端 的 实现	【face_detect_service.py】
1. 图片 的 两种格式：
  - ROS 图片格式：		消息接口sensor_msgs
  - OPENCV 图片格式
2. 两种 格式之间 需要进行 转换：通过**CV BRIDGE**进行 转换
```c
库： from cv_bridge import CvBridge
````
##### 服务实现 的 步骤：
1. 创建 服务类：在类中，创建 服务端`create_service()`；使用 回调函数，当接收到客户端request时，调用函数。
2. 回调函数：接收到消息后，调用`face_recognition`函数
3. 在 回调函数中 将 识别结果 进行处理，后 **合成 Response 并 返回 给 客户端**
#### （4）人脸检测 客户端 的 实现
1. 图片 的 两种格式：
  - ROS 图片格式：		消息接口sensor_msgs
  - OPENCV 图片格式
2. 两种 格式之间 需要进行 转换：通过**CV BRIDGE**进行 转换
```c
库： from cv_bridge import CvBridge
````
##### 客户端 实现步骤：
1. 定义 服务类，并在 服务类 中 创建客户端``
2. 构造 Request，给 服务端 发送请求
3. 处理 服务端 返回的 Response，并 绘制 人脸识别框 

### 3. 【C++服务通信项目】 实现 巡逻海龟
1. 需求： 让小海龟 在 海龟模拟器 中 随即游走 并且进行 巡逻
2. 分析：
- 怎么 改成 动态的目标点 接收	  服务端 接收 客户端发出的 目标点坐标，服务端指挥小海龟
- 用什么 通信接口？ 		       自定义接口
- 怎么实现 随机游走 ？			客户端 产生随机点，请求 巡逻服务
#### 创建 自定义接口
1. request请求值：
  - 目标点坐标：float x,y
2. response返回值：（**大写表示 常数**）
  - 服务端处理结果：int8 result 里，存放 成功/失败
  - 是否能走通。定义常量：SUCCESS、FAIL
3. 在 CMakelist中，添加 消息接口
```c
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Patrol.srv"
  DEPENDENCIES sensor_msgs
)
```
4. 构建 工作空间
5. 查看 自定义接口：`ros2 interface show 消息功能包/srv/消息接口文件`
#### C++ 服务端 实现（turtle_control.cpp）
1. 创建功能包
2. 
3. 
4. 查看 运行的服务 列表：`ros2 service list -t`
5. 检验：使用 rqt 图形界面。`rqt -> plugins -> service -> service call`
  - Servicce : 从列表中 选出 自定义的消息接口(/patrol)

#### C++客户端实现
实现步骤：
1. 创建 客户端 和 定时器
2. 定时 产生 目标点 并 请求 服务端巡逻

### 4.【Python 参数 通信项目】 人脸检测：将 采样次数 和 检测模型 参数化
#### 如何 参数化？
1. 把 **人脸检测参数 改为 ROS2参数 实现方法**：
- declare_parameter：用于 声明参数。
  - 第一个参数：参数 的 名称
  - 第二个参数：参数 的 默认值
- get_parameter(参数).value ：用于 获取 参数的值。.value 用于 获取其 真实值并赋给对应属性
  - 第一个参数：参数 的 名称
```py
def __init__(self):
	...
	# ROS2 参数化 声明
	self.declare_parameter('number_of_times_to_upsample',1)
	self.declare_parameter('model',"hog")
	# ROS2 获取 参数 / 设置 参数
	self.number_of_times_to_upsample = self.get_patameter('number_of_times_to_upsample').value
	self.mode = self.get_parameter('mode').value
```
2. 参数 相关 代码：
  1) ros2 param list ：查看 参数 列表
  2) ros2 param set /节点名字 参数名字 参数值：设置 对应的 参数 
  3) ros2 param get 节点名字 参数名字：获取 对应的 参数
3. 在 ==启动时，可以 **直接 设置参数**==
语法：`ros2 run 功能包名 可执行文件节点名 --ros-args -p 参数名字:=参数值`
  - `--ros-args`：要 添加 ros 参数
  - `-p`：一个参数
#### 订阅 参数更新
1. 原理：相当于 **服务通信**，在 服务端 **回调函数 中 更新 参数**
  - 查看服务列表：会出现`/节点/set_parameters `
  - 服务`set_parameters`作用 ：设置 参数值
2. 返回值：rcl_interfaces/srv/SetParametersResult
3. 语法：
```py
# 导入 SetParametersResult消息接口 库
from rcl_interfaces.msg import SetParametersResult

def __init__(self):
	# 添加 设置参数 的 回调函数：当 参数更新时，ROS2会自动调用 该回调函数
	self.add_on_set_parameters_callback(self.parameter_callback)

# 设置 回调函数：在回调函数 中 遍历 参数，并 赋值 对应参数
 def parameters_callback(self,parameters):
        for parameter in parameters:
		    self.get_logger().info(f'参数{parameter.name}设置为{parameter.value}')
            if parameter.name == 'number_of_times_to_upsample':
                # 设置参数
                self.number_of_times_to_upsample = parameter.value  
            if parameter.name == 'model':
                self.model == parameter.value
        # 需要 返回：参数设置 是 成功or失败
        return SetParametersResult(successful=True)     # 成功
```
#### 【服务 实践】修改 其他节点 的参数（使用 客户端）
1. 基本原理：在 客户端 中 通过**请求服务 通信**来 完成 参数设置。
2. 消息接口：
`ros2 interface show rcl_interfaces/srv/SetParameters`
3. 服务与消息接口 库
- 导入 SetParameters服务
`from rcl_interfaces.srv import SetParameters`
- SetParameters为 复合消息接口，里面还有个 Parameter消息接口：
`from rcl_interfaces.msg import Parameter`

### 5.【C++ 参数服务通信 项目】小海龟
#### （1） C++参数 声明 与 设置：
1) 参数：小海龟 的 比例系数k\_ 与 最大速度max\_speed\_
2) 语法：
```c
// 在 类中 
class TurtleController:public rclcpp::Node{
public:
	// 在 构造函数中 声明 与 初始化
	TurtleController():Node("turtle_controller"){
	
		// 声明 参数
		this -> declare_parameter("k",1.0);
		this -> declare-parameter("max_speed",1.0);
		
		// 获取 参数 初始值
		this -> get_parameter("k",k_)
		this -> get_parameter("max_speed",max_speed_)
	
	}
}
```
3) 函数：
- `declare_parameter( "参数名" , 参数的默认值 )`
  - 作用： 声明 有 该 参数 可以设置
  - 该 函数 在 Node类中

- `get_parameter( "参数名" ，用于接收参数值的变量 )`
  - 作用：获取 当前 的 参数值，并将其 放入 变量中
  - 该 函数 在 Node类中
  - 参数：
    - ==用于接收参数值的变量：传入 成员变量 （引用/指针）中==
4) 在 rqt 中 修改 参数：
`rqt -> Plugins -> Configuration -> dynamic Reconfig`
#### （2）C++ 接收 参数 事件
1. 原因：由于 使用 rqt或其余方式，更改 参数值 **只会更改 ros2 内部的 参数值**，不会改 成员变量中的值。
2. 目的：把 ROS2 内部的参数值 **拷贝到** 类成员变量 中。
3. 方法：
1) 使用 服务通信：==**`/节点名/set_parameters`**来 设置 参数==
  - 当 设置 参数后，系统 会 自动调用 回调函数
2) 其 **返回值类型 为：`rcl_interfaces/srv/SetParametersResult`**
3) 在 服务端 声明并调用 **回调函数**：
- ==回调函数 在 每一次 设置参数 时 被 调用==
  - 在 **回调函数** 中，对 **`request`**中的 数据 进行处理。
  -  返回 一个 **`response`**
4. 代码 步骤：
1) 导入 消息接口库：
`#include"rcl_interfaces/msg/set_parameters_result.hpp"`
2) 使用 **using** 简化 下 消息接口
`using SetParametersResult = rcl_interfaces::msg::SetParametersResult;`
3) 在 类 private  成员变量中，使用 共享指针 声明/创建 参数回调函数变量
`OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;`
4) 在 构造函数 中；在 声明与获取参数 后 ，对 变量 进行 初始化
`parameter_callback_handle_ = this->add_on_set_parameters_callback(回调函数);`
  - 使用的 方法：add_on_set_parameters_callback(回调函数)
    - 回调函数：使用 lamda 表达式 
5) 编写 回调函数：
```cpp
// 对 参数回调 变量 进行 初始化
        // 编写 回调函数
parameter_callback_handle_ = this->add_on_set_parameters_callback(
    [&](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
        // 创建 result 对象，用来 存储 获取参数 的 结果
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        // 使用的 是 vector数组：遍历 数组
        for (const auto & parameter : parameters){
            // 如果 成功遍历，打印 当前 处理的 参数名字
            // 因为 使用的是 C格式 下的 打印，字符串 需转换下 格式：c_str()
            RCLCPP_INFO(this->get_logger(),"更新参数的值: %s = %f",parameter.get_name().c_str(),parameter.as_double());
            // 通过 参数 的 名字 来 对 成员变量 进行赋值
            if(parameter.get_name() == "k"){
                // 获取 参数 中的 double类型数据
                this->k_ = parameter.as_double();
            }
            if(parameter.get_name()=="max_speed"){
                this->max_speed_ = parameter.as_double();
            }
        }
});
```
6) 还可以 通过 文件内的 代码，来 改变 自身的值【不常用】:（在 获取参数 下）
`this->set_parameter(rclcpp::Parameter("参数名",待设置的参数值));`
#### （3）（客户端） 修改 其他节点（服务端） 的 参数
1. 原理：创建 服务 客户端 ，使用 服务通信，请求 另外的节点 进行更新
2. 方法：调用 **`/节点/set_parameters`** 来 更新 节点 参数
3. 消息接口：`rcl_interfaces::srv::SetParameters`
4. 编写 代码：【在 客户端 cpp文件中 修改】
1) 导入 消息接口 库：(通过`ros2 service list -t`可以查看 set_parameters 的 消息接口)
```cpp
#include"rcl_interfaces/msg/parameter.hpp"
```
2) 导入 rcl_interfaces/msg/parameter 下的 嵌套/复合库：【总共 3个 消息接口】
（通过`ros2 interface show rcl_interfaces/msg/parameter`来查看）
  - `parameter_value` 消息接口  
  - `parameter_type`消息接口
  - `/srv/set_parameters` 消息接口
```cpp
#include"rcl_interfaces/msg/parameter_value.hpp"
#include"rcl_interfaces/msg/parameter_type.hpp"
#include"rcl_interfaces/srv/set_parameters.hpp"
```
3) 使用 using 来 简化 消息接口 命名空间
`using SetP = rcl_interfaces::srv::SetParameters;`
4) 在 类中，创建 客户端相关 的 成员函数：`call_set_parameter()`
- 函数内容：创建 客户端，发送 request请求，返回 response的共享指针 结果
  - ==返回类型：`SetP::Response::SharedPtr`==
  - 形参列表：`rcl_interfaces::msg::Parameter &param`
    - 原因： 由于 请求对象的数据 需 外部传入。所以，形参为 请求部分需要的数据parameter
```cpp
    SetP::Response::SharedPtr call_set_parameter(rcl_interfaces::msg::Parameter &param){
        // 1. 创建 客户端对象
        auto param_client = this->create_client<SetP>("/turtle_control/set_parameters");   // 通信 名字
        // 2. 检测 服务是否上线
        while (!param_client->wait_for_service(1s))
        {
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(),"等待服务上线过程中，rclcpp 挂了，退出程序");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"等待服务上线中......");
        }
        // 3. 构造 request请求 的 智能指针 对象
        auto request = std::make_shared<SetP::Request>();
        // request中 是有 参数数组Parameter[]:  需将 请求的数据param 添加至 参数数组中 
        request -> parameters.push_back(param);                 // C++数组中，使用 push_back 来给 数组 添加数据

        // 4. 发送请求【同步修改】：发送异步请求，等待请求结果；收到结果后，存入response 并 将其返回
        auto future = param_client -> async_send_request(request);                  // 异步 发送 request请求
        // spin_until_future_complete()方法:
        //      参数：1.节点共享指针get_node_base_interface         2. 发送请求对象
        rclcpp::spin_until_future_complete(this->get_node_base_interface(),future);  
        // 5. 发送请求后，获取 响应结果 放入 response对象中
        auto response = future.get();
        // 6. 返回 结果 指针
        return response;
    }
```
使用的方法：
  - this->create_client<消息接口>("消息名称")：创建 客户端
  - 数组.push_back(参数)：将 参数 添加至 数组中
  - rclcpp::spin_until_future_complete(节点指针，请求对象)：发送 请求
    - 节点指针：this->get_node_base_interface()
5) 在类中，创建 更新参数 的 成员函数：`update_server_param_k()`
- 函数作用：外部 可以 调用该函数 来 更新 某个参数值
- 形参列表：参数值K `double k`
```cpp
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
6) 在 主函数 中，调用 更新参数 方法：
```cpp
// 调用 更新参数函数
    node -> update_server_param_k(4.0);
```

## 九、 使用 launch 来 启动脚本
1. 作用：可以==在 脚本中 **一次启动 多个 节点**==
2. launch工具：ROS2中 用于 启动和管理 ROS2 节点与进程的工具
3. launch文件 的 格式：**python**，XML，YAML 三种
4. ==使用 debug 查错 launch：**`ros2 launch 出错的命令 --debug`** 
### 1. launc 文件的编写：【python 格式（常用）】
#### （1）使用 launch 启动 多个节点
1) 在 功能包文件夹 下 新建 `launch文件夹`，并创建`文件名.launch.py`的文件
2) 调用 库文件：
```py
import launch
import launch_ros
```
3) 程序语法：
- 定义函数：**`def generate_launch_description():`（注意：【函数名不能更改】）**
- 返回值：为**launch的描述：`launch.LaunchDescription([actions数组])`
```py
# 1. 创建 actions 对象,并 传入参数
    # 一次 运行 3个 节点/动作
    action_node_turtlesim_node =  launch_ros.actions.Node(
    # 传入的参数为：
        # (1) 功能包 的名字
        package = 'turtlesim',   
        # (2) 可执行文件 的 名字        
        executable= 'turtlesim_node',             # 小海龟 节点
        # (3) 日志 输出 的 目的地  
        output = 'screen',                        # screen：输出 至 屏幕 
    )
    action_node_patrol_clien =  launch_ros.actions.Node(
        package = 'demo_cpp_service',      
        executable= 'patrol_client',              # 巡逻 客户端 节点
        output = 'log',                           # log：输出 至 日志文家          
    )
    action_node_turtle_control =  launch_ros.actions.Node(
            package = 'demo_cpp_service',        
            executable= 'turtle_control',         # 小海龟 控制 服务端 节点
            output = 'both',                      # both：两者 都输出     
        )
    # 2. 返回 LaunchDescription 对象，其 参数为一个 数组
    return launch.LaunchDescription([
        # 数组 内容为：actions动作
        action_node_turtlesim_node ,                    # 启动 小海龟
        action_node_patrol_clien ,                      # 启动 巡逻 客户端
        action_node_turtle_control                      # 启动 控制 服务端
    ])
````
#### （2）使用 launch 传递 参数
1. ros2 方法 来 传递参数：
`ros2 run turtlesim turtlesim_node --ros-args -p 参数:=参数值`
2. 使用 launch文件 手动 修改 参数：
  1) 声明 一个 `launch.actions` 对象 ：
- 方法：`launch.actions.DeclareLaunchArgument(参数)`
  - 参数：（1）给参数起一个别名 	（2）默认值:default_value = '值' （必须是 字符串格式）
```py
def generate_launch_description():
	action_declare_参数对象名字= launch.actions.DeclareLaunchArgument('参数别名',default_value='默认值')
```
  2) 把 launch 的参数 手动传递 给 某个节点
- `launch.substitutions.LaunchConfiguration('参数别名')` 
  - 函数 作用：把 launch中的 参数 转换成 节点中用的参数
  - **参数别名 作用**：在 命令行 中**使用 该别名 可对 节点中参数 进行修改**
```py
	# 在某一节点中：
	action_node_节点名 = launch_ros.actions.Node(
		# 将 launch中的参数 转化为 节点内的参数
		parameters =
[{'参数名':launch.substitutions.LaunchConfiguration('参数别名')}],
	)
```
  3) 在 return 中 加入`launch.actions` 对象：
```py
	return launch.LaunchDescription([
		action_declare_参数对象名字
    ])
```

### 2. launch 文件路径 的 配置
1) 【C++中】MakeLists.txt的编写：
```txt
# 将 launch 添加 至 install路径中
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)
```
2) 【Python中】setup.py 的 编写
  - glob函数：可以利用 **匹配符** 自动生成列表 放入 share目录中
```py
# 1. 导入 glob库 中 的 glob类
from glob import glob

# 2. 在 data_files 中：导入 一个 glob函数： 生成一个列表
data_files=[
	('share/' + package_name + "/launch", glob('launch/*.launch.py'))
]
```
### 3. 运行 launch 文件 的 终端命令
1. 使用 ros2 命令：**`ros2 launch 功能包名 文件名.launch.py`**
2. 在 文件夹中 直接使用： 需 提前 `source 文件夹下的setup.bash`
3. **设置 参数 的 命令：`ros2 launch 功能包名 文件名.launch.py 参数别名:=值`**

### 4. Launch 使用 进阶
1. Launch 的 三大组件：
- ==动作==：除了是 节点外，还可以是 打印、终端指令、甚至是 另一个launch文件
- ==替换==：使用 launch的参数 **替换** 节点中的参数值
- ==条件==：利用 条件可以决定 哪些动作启动，哪些不启动。（** 相当于if **）
#### Launch 动作 组件：
1. 使用 其他 launch文件：
1) 导入  库：
- `launch.launch_description_sources`
- `get_package_share_directory`
  - 功能：使用 该库 中 `get_package_share_directory`函数 来获取 其他launch文件的路径
```py
# launch 库
import launch.launch_description_sources
# 通过 功能包名字 获取 share 目录
from ament_index_python.packages import get_package_share_directory
```
2) 程序：
- 用 数组 组成 路径：
**`get_package_share_directory('功能包名'),'/launch','.launch.py'`**
- `get_package_share_directory('功能包名')`：寻找 功能包 路径
- `launch.actions.IncludeLaunchDescription`：寻找 launch文件
- `launch.launch_description_sources.PythonLaunchDescriptionSource`：拼接路径
```py
def generate_launch_description():
	# 1. 创建 路径数组 
	multisim_launch_path = [get_package_share_directory('turtlesim'),'/launch','/multisim.launch.py']
	# 2. 创建 action对象：通过 launch文件路径 寻找 launch文件
	action_include_launch = launch.actions.IncludeLaunchDescription(
launch.launch_description_sources.PythonLaunchDescriptionSource(multisim_launch_path)
	# 3. 返回 动作action（多个 动作，用 ， 隔开）
	return launch.LaunchDescription([
		action_include_launch
	])
    )
```
2. 执行 终端命令行：
- `launch.actions.ExecuteProcess(cmd=['命令字段1','命令字段2',...])`
```py
def generate_launch_description():
    # 创建 action 对象：执行 命令行语句
    action_topic_list = launch.actions.ExecuteProcess(
        cmd=['ros2','topic','list']   		# 执行ros2 topic list
    )
    return launch.LaunchDescription([
		action_topic_list
	])
```
3. 输出 日志：
- `launch.actions.LogInfo(msg=str(对象))`：打印 对象 内容
```py
def generate_launch_description():
	# 创建 action 对象：打印 内容
	action_log_info = launch.actions.LogInfo(msg=str(multisim_launch_path))
	
	return launch.LaunchDescription([
		action_log_info
	])
```
4. 使用 定时器动作：
- `launch.actions.TimerAction(period=时间,actions=[action对象])`
- 作用：在 第{时间}s后，执行 动作action
5. 组织 多个动作 放在一起，==**按顺序**依次实现==
- `launch.actions.GroupAction([动作1,动作2,...])` 
- 作用：**依次执行 动作/定时器动作**
```py
def generate_launch_description():
	# 创建 action对象：组织动作
    action_group = launch.actions.GroupAction([
    # 组织 多个动作 的 先后顺序

    # 使用 定时器 间隔 执行 动作
    # 第2s后，启动 action_include_launch
    launch.actions.TimerAction(period=2.0,actions=[action_include_launch]),     
    # 第4s后，启动 action_topic_list
    launch.actions.TimerAction(period=4.0,actions=[action_topic_list]) 
])
	return launch.LaunchDescription([
        # 这两个 动作 会 一起 执行
        action_log_info,
        action_group
    ])
```
#### Launch 条件 组件：
1. 需要 通过 **参数 传递**来实现 条件 功能：
1) 声明 参数 对象：
- 该参数：为bool类型，True or False
```py
action_declare_参数 = launch.actions.DeclareLaunchArgument('参数别名',default_value='True/False')
```
2) 创建 **参数传递** 对象，使用 该对象 与 条件 即可实现：
```py
参数传递对象 = launch.substitutions.LaunchConfiguration('参数别名')
```
3) 在 **具体方法**中，创建 条件 组件 参数
- `launch.conditions.IfCondition(参数传递对象)`
- 相当于：if (参数传递对象){ 执行 行动 }
```py
	action_对象 = launch.actions.动作(
		# 创建 条件 组件对象
		condition = launch.conditions.IfCondition(参数传递对象),
		其他 组件
	) 
```
2. 举例 程序：
```py
def generate_launch_description():
	# 1. 创建 参数声明 对象，声明 launch参数，以 后面调用
	action_declare_startup_rqt = launch.actions.DeclareLaunchArgument('startup_rqt',default_value='False')
	# 2. 创建一个 参数传递 对象，使用该对象 加上 条件 即可控制
    startup_rqt = launch.substitutions.LaunchConfiguration('startup_rqt')
    # 3. 在具体 行动中 加入 参数 条件
     action_topic_list = launch.actions.ExecuteProcess(
        # if startup_rqt:   
        #    run rqt
        condition = launch.conditions.IfCondition(startup_rqt),
        cmd=['rqt']                         
    )
    return launch.LaunchDescription([
        action_declare_startup_rqt
    ])
```
3. 使用 launch时，设置条件：
`ros2 launch 功能包名 名.launch.py 参数别名:=False/True`

## 十、常用 工具


## 十一、建模
### 10.1 机器人建模 结构
1. 结构：
![image-20250814175235648](/home/zyx/snap/typora/102/.config/Typora/typora-user-images/image-20250814175235648.png)
2. 常用的 方针平台：
- Gazebo click
- weballs
- Unity3D
- Matlab/Simulink
### 10.2 机器人建模 文件格式 【URDF】
1. URDF 使用 XML（可扩展标记语言）来 描述 机器人的 几何结构、传感器和执行器等 信息。
2. 文件语法：
```c
// 这是xml的版本
<?xml version="1.0"?>
// robot 是 一个 标签
<robot name="first_robot">
	<!--- 这是 XML语言 的 注释	--->
	<link name="base_link"></link>		// link 为 robot的子标签
</robot>
// 创建一个，名字为 first_robot的机器人，内部有 base_link部件
```
3. 创建 URDF 功能包：（不需要 加入依赖）
4. 在 功能包 下，创建 urdf文件夹，用来存放 urdf文件
#### URDF 语法
#### URDF 工具
##### urdf_to_graphviz
1. 该 工具 的 作用： 通过 URDF文件 生成 **pdf 与 gv格式 的文件**
2. 语法：==**`urdf_to_graphviz urdf文件.urdf`**== 


# ROS2常见的错误
### package '...' was not found：可能没有配置环境变量 或 环境变量下没有该功能包
解决办法：
（1）查看环境变量（AMENT_PREFIX_PATH）下的文件有没有该功能包
（2）若没有，则添加功能包 或者 使用source添加环境变量

## 查看 ros2 topic echo /话题名称 时，报错：
### 错误：The message type '话题类型' is invalid
- 原因：当前命令窗口 的 环境变量 没有 更新
- 解决办法：在 工作空间 目录下 使用 `source intall/setup.bash`


# 功能包 结构目录
## 1. build
## 2. package名字
### package文件夹
用途：放置 节点代码 的文件夹。【开发文件的主战场】
#### \_\_init\_\_.py 文件
为 python包 的标识文件。
（若包含该文件，则为 一个python包）
### resource 文件夹（一般不用管）
用途：放置一些资源。提供 功能包的标识。
### test 文件夹
用途：测试代码文件夹。存放 测试代码 的文件。
  - 对 package中的代码，进行测试并生成报告
### LICENSE
功能包的许可证。用于 保护知识产权用。【一般使用 Apache-2.0】
### package.xml
声明了 功能包的 名称、版本编号、构建类型、许可证等信息。
### setup.cfg
存放 python包的 配置选项。
### setup.py
构建脚本文件。包含 setup函数，用于指定 如何构建功能包，声明 可执行文件的名称等

## 3. install
用途：用于 存放 构建结果 的文件夹
### 1. package 名字
#### lib
##### package 名字
用途：存放 生成后的==可执行文件==。
##### python版本
用途：在此处 可查看 **源文件程序**。
原因：系统 将源文件 拷贝一份放到 该目录喜下的package文件夹中。
重点：==系统运行的是 该目录下的 源函数文件==
#### share
## 4. log

# 做测试
1. 做测试时，可以先将 单独的文件进行 编译。
  - python：`python3 文件名`
  - C++：`./a.out（或 可执行文件名）`

2. 测试好后，再将其放入 工程文件夹中 进行构建。

# ROS2语法
## ros2 run
## ros2 pkg : 功能包相关
### create
### prefix：
```c
	ros2 pkg preflex <功能包名>		// 查看 功能包的路径
```
## ros2 interface : 消息接口相关
### ros2 interface list：查看 功能包内 有什么文件
### ros2 interface show：查看 功能包 内 文件的 具体内容
1. 功能 ： 查看 功能包的 具体内容。`ros2 功能包 show 具体文件`

如： 查看 example_interfaces/msg/String 的内容
```c
ros2 interface show example_interfaces/msg/String
```
## ros2 service：消息 服务 相关
### ros2 service list -t : 查看 当前 运行的 服务
### ros2 service call 服务名字 服务接口 "消息内容"： 给 指定的 服务 发送 消息

## ros2 param : 参数 服务 相关
### ros2 param list ：查看 参数 列表
### ros2 param set /节点名字 参数名字：	 设置 对应的 参数
### ros2 param get /节点名字 参数名字：	获取 对应的 参数


# 编程基础
## 一、面向对象编程
1. 定义：
2. 三大特性：
  - ==封装==
  - ==继承==
  - ==多态==
3. 方法 与 函数：【通过==点运算==区分】（在 类 中）
  - 方法： 类.函数名()
  - 函数：直接定义 函数名

4. 
### 1. 类
1. 第一步： 是创造一个==类==（包含 属性 和 方法）【定义类：class】
  - 属性：**属性不能重复**
  - 方法：**方法可以共用**
```py
class PersonNode:	# 只是个 类，还不是对象
	# 定义 类的属性
	def__init__(self,name:str,age:int)->None: 
		self.age = age;
		selg.name = name;
	# name:str 说明 name 是 str类型
	# age:int 说明 age 是 int类型
		
	# 定义 类的方法
	def eat(self,food_name:str):
		print('')
```
```c
// 定义 一个类class
class PersonNode : public rclcpp::Node
{
    // 声明 成员数据（属性）【私有 成员】
private: // 成员声明： 数据类型  成员名      ； 加入 下划线 表示为 私有成员
    std::string name_;
    int age_;

    // 声明 成员函数（方法、构造函数）【共有 函数，可被调用】
public:
    PersonNode(const std::string &name, const int age)
    {
        // 对 成员数据（属性） 进行 赋值: 使用 this -> 属性 = 形参      [this 表示 这个 类]
        this->name_ = name;
        this->age_ = age;
    }

    // 声明 方法
    void eat(const std::string &food_name)
    {
		std::cout << food_name << std::endl;
    }; // 短函数，可在 类中定义（结尾需加分号；）

}; // 必须有 分号；

```
2. 第二步：对==类进行**实例化**==【基于 类的模板，使用自己的参数，来生成一个对象】
```py
	# 生成 实例化对象 node
	node = PersonNode('张云旭','25')		// 里面是 具体的 类的属性
	
	# 调用 对象的方法
	node.eat('鱼香肉丝')
```
### 2. 继承
1. 优点：使用 继承 可以 很方便创建 基于一个类的 新的类
2. 方法：
```py
# 从 rclpy功能包中的rclpy_node节点文件内，导入 Node类
import rclpy
from rclpy.node import Node

# 创建一个 PersonNode类
class PersonNode(Node):	# 在（）内，添加待继承的父类Node
	# 定义 属性（包含父类的属性）
	def __init__(self,node_name:str,name:str,age:int) -> Node:		
		# 从 父类(super)中，继承父类的__init__方法内的属性
		super().__init__(node_name)	# 并且 传入 node_name参数
		
		# 创建 子类中 独特的 属性
		self.name = name
		self.age = age
	
	# 创建 与 继承 类的方法
	def eat(self,food_name:str)			# 创建 PersonNode类的方法
		# 已经继承了 父类，所以 使用 self 可直接调用父类的方法
		self.get_logger().info(f"{self.name}今天过了自己的{self.age}岁的生日，并且吃了{food_name}")         
```
```py
def main():
    rclpy.init()        # 初始化节点,分配资源
    
    # 实例化：创建对象node
    node = PersonNode('jiedian','zyx',25)
    # 使用对象，调用方法
    node.eat('蛋糕')

    rclpy.spin(node)    # 运行节点对象，并收集事件与执行
    rclpy.shutdown()    # 关闭节点，并清理内存
```
```c
// 定义 一个类class
class PersonNode : public rclcpp::Node
{ // 继承 使用 public；继承 rclcpp包 下的 Node类

    // 声明 成员数据（属性）【私有 成员】
private: // 成员声明： 数据类型  成员名      ； 加入 下划线 表示为 私有成员
    std::string name_;
    int age_;

    // 声明 成员函数（方法、构造函数）【共有 函数，可被调用】
public:
    // 设置成员数据，通过初始化列表(onst std::string &node_name)，来显式地调用 父类Node()的 成员数据/属性
    PersonNode(const std::string &node_name, const std::string &name, const int age)
        : Node(node_name) /* 调用 父类 的 构造函数*/ // 将PersonNode获取的node_name，传入 父类Node中
    {
        // 对 成员数据（属性） 进行 赋值: 使用 this -> 属性 = 形参      [this 表示 这个 类]
        this->name_ = name;
        this->age_ = age;
    }

    // 声明 方法
    void eat(const std::string &food_name)
    {

        // 使用 node父类的方法: 使用 this 说明 get_logger()是 继承到该类中的
        RCLCPP_INFO(this->get_logger(),"我是%s,%d岁，爱吃%s", this->name_.c_str(), // name_.c_str() 将 char类型的name 转换成 str类型
                    this->age_, food_name.c_str());                        // name 与 age 都为 私有属性，所以用 this 来调用

    }; // 短函数，可在 类中定义（结尾需加分号；）

}; // 必须有 分号；
```
```c
int main(int argc,char **argv){

    // 初始化 节点
    rclcpp::init(argc,argv);
    
    // 创建对象:实例化 类
    // make_shared 将 node 变为一个 PersonNode类的指针  【智能指针】
    auto node = std::make_shared<PersonNode>("person_node","zyx",18);     // 将 PersonNode需要的参数(包括父类需要的)，都加上
    // 打印日志
    RCLCPP_INFO(node->get_logger(),"hello c++ node");
    
    // 运行 eat
    node -> eat("鱼香肉丝") ;               // 使用 -> 来 调用 成员函数（方法）

    // 运行 节点
    rclcpp::spin(node);
    // 清理内存
    rclcpp::shutdown();
    
    return 0;
}
```
## 二、C++ 的新特性
### 1. 自动类型推导 auto
1. 自动识别类型：`auto 变量 = 表达式`
2. 根据 表达式，自动推导变量的类型，并且 赋予类型
```c
	auto node = ...
```
### 2. 智能指针 
1. 语法：`make_shared<类>(参数)`
2. 使用 该语法，就能创造出一个指针
3. 作用： 可以 **管理 动态分配的内存、避免 内存的泄漏、空指针**等问题。
4. C++11，提供了 三种 智能指针：
  - unique_ptr
  - ==shared_ptr 共享指针==
  - weak_ptr
##### 共享指针
1. 作用：每用一次，都会==创建一次 **引用计数**（引用数量+1）==。当使用一次引用，就会减1,==使用完会 **自动释放内存**==。

2. 本质：开辟一个堆区，用完后可以自动释放。（new 是 手动释放）
3. **头文件**：`#include<memory>`
4. 语法：`std::make_shared<数据类型/类>(参数)`
  - 参数：为 创建该 **数据类型/类 所需的 参数（实参）**
  - **返回值**：为 ==对应类中**参数的共享指针**==【指向 参数的 地址】
5. **释放引用**：`对象.reset()`
6. 查看 相关功能：
  - 查看 引用次数：`对象.use_count()`
  - 查看 指针指向的地址：`对象.get()`
  - 查看 指向的内存地址**具体数据**：`对象 -> c_str()`
```c
#include<memory>

// make_shared<数据类型/类>(参数)
// 其中，参数：为 创建该 数据类型/类 所需的 参数
// 返回值：为 对应类中 参数 的共享指针
int main(){
	// 创建对象p1.此时，p1指向"this is a str"的地址 被引用了一次
	auto p1 = std::make_shared<std::string>("this is a str");	
	
	// 使用 .use_count() 来 得到 指向内存地址的 引用次数
    std::cout << "p1 的 引用计数" << p1.use_count() << std::endl; // 为 1

    // 使用 .get() 来 获取 共享指针的地址(内存地址 为 p1指向的地址)
    std::cout << "指针指向的内存地址为" << p1.get() << std::endl;   
	
	// p1 指针 调用成员方法c_str()将内存地址数据 用字符串打印出来
	 std::cout << "p1 指向的内存地址 数据：" << p1 -> c_str() << std::endl;  // 打印为 this is a str
     
	// 释放内存
	p1.reset();// 此时，p1不再指向"this is a str"地址，引用次数为0,地址也为0。				
}
```
7. 优点：
- 无论 智能指针 进行 拷贝或者分享 多少份，其 **内存地址都不变**。==运行效率会提高==。
- 当所有程序 使用完内存地址 之后，会**自动的回收**。==不会造成内存泄漏==

### 3. Lambda 表达式（定义函数 的一种方法）
1. 性质：**匿名函数**（没有名字）
2. 头文件：`#include<algorithm>`（算法库）
3. 语法：`[capture list](parameters) -> return_type{ function body }`
  - capture list：捕获参数列表（将捕获到的变量 传至 { }作用域中）
    - 当 为 ==[&]==：表示 捕获 上限文中所有的变量（都可在作用域中使用） 
  - parameteres：函数的参数列表。（形参列表）
  - -> ：指针 指向 返回值
  - return_type：函数 返回值的类型
  - {function body}：函数体
```c
	#include<algorithm>
	int main(){
		auto 对象 = [capture](parameter) -> return_type { 表达式; };
	}
```
### 4. 函数包装器（可以保护数据）
1. C++中的 **三种函数**：
  - ==自由函数==： 自己定义的函数
  - ==成员函数==：在 类的内部
  - ==Lambda函数==：
2. 函数包装器 的 功能：为了**统一 这三种 函数**。（更安全）
#### 包装
1. 函数包装器 的 **头文件**：`#include<functional>`
2. 函数包装器 的 **代码**：
    ==`std::function<返回类型(形参的类型列表)> 包装后的对象 = 函数名`
  - 其中，<> 中 放的是 函数模板
  - 若为**成员函数**，则需要使用 bind绑定。
3. 类的成员函数 需要通过 **bind绑定**：`std::bind(类指针，对象指针，占位符)`的参数
  - **类的指针**：类 对应的方法(模板)的 指针/地址。==`&类名::成员函数名`==
  - **对象的指针**：对象 的 指针/地址。==`&类的对象`==，可以用**`this`来代替**
  - **占为符 列表**：假如 有 n个数量的形参。注意：**从1开始,2,3,直到n**
  - ==`std::placeholders::_1,std::placeholders::_2,...`== 
4. 举例：
```c
// 导入 函数包装器 的 头文件
#include<functional>

// 将 自由函数save_with_freefun 包装成 save1   
std::function<void(const std::string& )> save1 = save_with_freefun;  
 // 将 lambda函数save_with_lambda 包装成 save2
std::function<void(const std::string& )> save2 = save_with_lambda;     
// 将 file_save类 的 成员函数save_with_class 放入 包装器:【需要 绑定bind】
std::function<void(const std::string& )> save3 = std::bind(&FileSave::save_with_class,&file_save,std::placeholders::_1);
```
#### 调用 函数包装器
1. 语法：`包装后的对象(要传入的实参);`
2. 比如：
```c
 	// 调用 包装函数
    save1("file.txt");
    save2("file.txt");
    save3("file.txt");
```
## 三、 多线程 与 回调函数
1. 多线程 Multi-threading ：多个程序，并行运行
2. 回调函数 ： 当多线程的任务完成后，传递给 回调函数，哪个任务完成，就传递给哪个。（省时）
3. 多线程的缺点：线程数量过多，会影响 系统的调度
4. 注意：在 ros2中，往往用单线程。当 逻辑运算多时，用 多线程 
4. 程序：Python
```py
def 函数(self,形参):
	# 创建对象，创建一个新的线程。 target 线程启动后执行的目标函数, args 为 目标函数的形参
	thread = threading.Thread(target=目标函数,args=(目标函数的形参))
	# 启动线程
	thread.start()

def main():
	# 创建对象
	对象 = 函数()
	
	# 使用 线程
	对象.函数(形参)

```
5. 程序：C++
  - 头文件：`#include<thread>`
  - 与 时间 相关的 头文件：`#include<chrono>`

