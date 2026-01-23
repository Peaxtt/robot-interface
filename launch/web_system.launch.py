#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. หาตำแหน่งโฟลเดอร์ปัจจุบันของไฟล์นี้ (launch)
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    # 2. ถอยหลัง 1 ขั้น เพื่อไปที่ root ของ 'robot-interface'
    project_root = os.path.dirname(current_dir)

    # 3. ระบุตำแหน่งไฟล์ Bridge และ Config แบบ Relative
    script_path = os.path.join(project_root, 'bridge', 'action_bridge.py')
    
    # หมายเหตุ: ถ้า Config ยังอยู่ที่ facobot_description ให้แก้ path ตรงนี้เป็น path เต็มแทน
    # แต่ถ้าคุณย้าย config มาไว้ใน robot-interface/config/ แล้ว ก็ใช้บรรทัดล่างนี้ได้เลย
    # config_path = os.path.join(project_root, 'config', 'web_action_params.yaml')
    
    # (Fallback) ถ้า Config อยู่ที่ facobot_description
    config_path = '/home/admin/facobot_ws/src/facobot_description/config/web_action_params.yaml'

    return LaunchDescription([
        Node(
            package=None, # ไม่ใช้ Package
            executable=script_path, # รันไฟล์นี้ตรงๆ
            name='web_action_bridge',
            output='screen',
            parameters=[config_path]
        )
    ])