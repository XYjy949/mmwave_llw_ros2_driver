cmake_minimum_required(VERSION 3.8)
project(novatel_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# 查找依赖
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 标准消息
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 自定义 msg 生成依赖
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =========================
# 生成自定义消息
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NetParam.msg"
  "msg/RadarMountInfo.msg"
  DEPENDENCIES builtin_interfaces
)

# =========================
# 可执行节点
# =========================
add_executable(novatel_node
  src/novatel_node.cpp
  src/udp_interface.cpp
)

# =========================
# 依赖链接
# =========================
ament_target_dependencies(novatel_node
  rclcpp
  std_msgs
  nav_msgs
)

# 链接自定义 msg 的 typesupport
target_link_libraries(novatel_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# =========================
# 安装
# =========================
install(TARGETS novatel_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

