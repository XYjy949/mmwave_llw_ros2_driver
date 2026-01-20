cmake_minimum_required(VERSION 3.8)
project(novatel_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
#寻找依赖
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(novatel_oem7_msgs REQUIRED)  # 关键！
include_directories(
  ${novatel_oem7_msgs_INCLUDE_DIRS}
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

#将自定义的msg生成对应的头文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NetParam.msg"
  "msg/RadarMountInfo.msg"
  DEPENDENCIES builtin_interfaces
)

#添加可执行文件
add_executable(novatel_node 
  src/novatel_node.cpp
  src/udp_interface.cpp
  src/udp_interface.h
)

#连接ros2库
target_link_libraries(novatel_node PUBLIC 
  rclcpp::rclcpp
  ${novatel_oem7_msgs_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

link_directories(
  ${novatel_oem7_msgs_DIR}/../../../lib
)

#加载novatel的连接库
target_include_directories(novatel_node PUBLIC
  ${novatel_oem7_msgs_INCLUDE_DIRS}
)

#安装导出
install(TARGETS novatel_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
