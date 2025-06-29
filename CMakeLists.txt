# 1.声明CMake文件：CMake版本最低是3.8
cmake_minimum_required(VERSION 3.8)

# 2.工程的配置：主要配置工程的名字
project(<工程名字>)

# 3.添加可执行executable文件：将cpp文件 编译 生成 该可执行文件
add_executable(<待生成的可执行文件的名字> <待编译的cpp文件名字>)

# 4.直接查找到 头文件和库文件 并引入: 找到需要用的头文件
find_package(<需要查找的头文件夹名> REQUIRED)

# 打印内容
meassage(STATUS ${头文件或库文件(大写)的路径})

# 5.将找到的头文件，与 待编译的c++文件 连接
target_include_directories(<待生成的可执行文件的名字> PUBLIC $<要查找的头文件的路径>)   # 如：${rclcpp_INCLUDE_DIRS}

# 6.连接 库文件
target_link_libraries(<待生成的可执行文件的名字> $<要查找的库文件路径>)                 # 如：${rclcpp_LIBRARIES}