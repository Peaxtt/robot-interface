#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. ระบุตำแหน่ง Config ที่เราสร้างไว้
    pkg_description = get_package_share_directory('facobot_description')
    bridge_config = os.path.join(pkg_description, 'config', 'web_action_params.yaml')

    return LaunchDescription([
        # =========================================================
        # เราเปิดแค่ Action Bridge เท่านั้น!
        # เพราะ:
        # - Driver/Lidar/Odom -> เปิดโดย system_bringup แล้ว
        # - Rosbridge (WebSocket) -> เปิดโดย system_bringup แล้ว
        # =========================================================
        Node(
            package='facobot_description',
            executable='action_bridge.py',
            name='web_action_bridge',
            output='screen',
            parameters=[bridge_config] # โหลดค่า Config (Ratio/Speed)
        )
    ])