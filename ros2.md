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

# ROS2常见的错误
## package '...' was not found：可能没有配置环境变量 或 环境变量下没有该功能包
解决办法：
（1）查看环境变量（AMENT_PREFIX_PATH）下的文件有没有该功能包
（2）若没有，则添加功能包 或者 使用source添加环境变量


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