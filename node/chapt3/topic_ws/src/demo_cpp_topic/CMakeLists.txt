cmake_minimum_required(VERSION 3.8)
project(demo_cpp_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 1. 寻找 使用的库
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# 2. 添加 可执行文件
add_executable(turtle_circle src/turtle_circle.cpp)
# 3. 给 可执行文件 添加 依赖
ament_target_dependencies(turtle_circle rclcpp geometry_msgs) 

add_executable(turtle_control src/turtle_control.cpp)
ament_target_dependencies(turtle_control rclcpp geometry_msgs turtlesim)  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 4. 将 可执行文件 安装 到 install/lib 中
install(TARGETS turtle_circle turtle_control
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
