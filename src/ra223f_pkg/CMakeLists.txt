cmake_minimum_required(VERSION 3.20)
project(ra223f_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#寻找依赖
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)  # 必须添加

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
 
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

# 添加依赖
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectAtt.msg"
  "msg/ObjectList.msg"
  "msg/PointCloud.msg"
  "msg/PointCloudList.msg"
  "msg/RadarStatus.msg"

  DEPENDENCIES builtin_interfaces sensor_msgs visualization_msgs  
)

#添加可执行文件
add_executable(ra223f_node 
  src/ra223f_node.cpp
  src/udp_interface.cpp
  src/udp_interface.h
)

#连接ros2库
target_link_libraries(ra223f_node PUBLIC 
  rclcpp::rclcpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

#安装导出
install(TARGETS ra223f_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
