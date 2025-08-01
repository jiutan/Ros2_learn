cmake_minimum_required(VERSION 3.8)
project(status_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)                     # 消息接口 功能包
find_package(rosidl_default_generators REQUIRED)

# Cmake 函数，来自 rosidl_default_generators 功能包：用于 将 msg等 消息接口 定义文件，转换成 库 或者 头文件类
#       函数：rosidl_genterators_interfaces(参数)
#       参数：【参数 之间 用 回车 隔开】
#             1. 工程名字：                    ${PROJECT_NAME}
#             2. msg消息接口 文件 的 相对路径：  "msg/消息接口文件.msg"  (若有 多个 ，在 后面写即可，用 空格/回车 隔开 即可)
#             3. 自定义 新的消息接口 需要的依赖： DEPENDENCIES 需要的依赖
rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/SystemStatus.msg"
  DEPENDENCIES builtin_interfaces

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

ament_package()
