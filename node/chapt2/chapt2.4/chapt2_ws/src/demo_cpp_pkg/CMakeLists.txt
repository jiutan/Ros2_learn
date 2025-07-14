cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)           # 必须有，才能打印
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# 在这里 添加 生成可执行文件 以及 连接的cpp文件
add_executable(cpp_node src/cpp_node.cpp)
add_executable(person_node src/person_node.cpp)


# message(STATUS ${rclcpp_INCLUDE_DIRS})  # 打印 rclcpp的头文件路径及rclcpp依赖的头文件
# message(STATUS ${rclcpp_LIBRARIES})     # 打印 rclcpp的库文件及rclcpp以来的库文件

# target_include_directories(cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS}) # 头文件包含
# target_link_libraries(cpp_node ${rclcpp_LIBRARIES})    # 库文件连接
# 下面 替代 上面的两句话

# # 在这里，连接 下库
ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(person_node rclcpp)



install(
  TARGETS cpp_node person_node                        # 在这里 添加 可执行文件，使其放入 lib 下
  DESTINATION lib/${PROJECT_NAME}
)

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

# 若不加这个，则可能 忽略ament的语法
ament_package()
