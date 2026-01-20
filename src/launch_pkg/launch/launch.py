from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ra223f_pkg',      # 包名
            executable='ra223f_node',  # 可执行文件名
            name='ra223f_node',        # 节点名称
            output='screen',           # 输出到屏幕
        ),

        Node(
            package='novatel_pkg',      # 包名
            executable='novatel_node',  # 可执行文件名
            name='novatel_node',        # 节点名称
            output='screen',           # 输出到屏幕
        )
    ])