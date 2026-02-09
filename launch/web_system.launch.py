#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. หาตำแหน่งโฟลเดอร์ปัจจุบันของไฟล์นี้ (launch)
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    # 2. ถอยหลัง 1 ขั้น เพื่อไปที่ root ของ 'robot-interface'
    project_root = os.path.dirname(current_dir)

    # 3. ระบุตำแหน่งไฟล์ Bridge และ Config แบบ Relative
    script_path = os.path.join(project_root, 'bridge', 'action_bridge.py')
    
    # (Fallback) ถ้า Config อยู่ที่ facobot_description
    config_path = '/home/admin/facobot_ws/src/facobot_description/config/web_action_params.yaml'

    # =========================================
    # 4. เพิ่ม: ROSBRIDGE SERVER
    # =========================================
    # ค้นหาตำแหน่งของ package 'rosbridge_server'
    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    
    # สร้างคำสั่ง launch สำหรับ rosbridge (ใช้ไฟล์ xml มาตรฐานของเขา)
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        # (Optional) ถ้าต้องการเปลี่ยน Port สามารถ uncomment บรรทัดล่างนี้ได้
        # launch_arguments={'port': '9090'}.items()
    )

    return LaunchDescription([
        # สั่งรัน rosbridge ก่อน
        rosbridge_launch,

        # แล้วค่อยรัน Bridge ของเรา
        Node(
            package=None, # ไม่ใช้ Package
            executable=script_path, # รันไฟล์นี้ตรงๆ
            name='web_action_bridge',
            output='screen',
            #parameters=[config_path]
        )
    ])