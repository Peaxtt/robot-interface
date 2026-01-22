import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def is_process_running(process_name):
    """ฟังก์ชันเช็คว่ามี Process ชื่อนี้รันอยู่ในระบบหรือไม่"""
    try:
        # ใช้ pgrep -f เพื่อเช็คชื่อ process ทั้งหมด
        output = subprocess.check_output(["pgrep", "-f", process_name])
        return len(output) > 0
    except subprocess.CalledProcessError:
        return False

def generate_launch_description():
    home = os.environ['HOME']
    scripts_dir = os.path.join(home, 'simulation_facobot', 'facobot_ws', 'src', 'scripts')
    urdf_file = os.path.join(scripts_dir, 'box_bot.urdf')
    rviz_config = os.path.join(scripts_dir, 'view_robot.rviz') 

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    launch_items = []

    # 1. เช็ค ROSBRIDGE (WebSocket Port 9090)
    if not is_process_running("rosbridge_websocket"):
        launch_items.append(
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
                ])
            )
        )
    else:
        launch_items.append(LogInfo(msg="⚠️ Rosbridge is already running, skipping..."))

    # 2. เช็ค ROBOT STATE PUBLISHER
    if not is_process_running("robot_state_publisher"):
        launch_items.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_desc}]
            )
        )
    else:
        launch_items.append(LogInfo(msg="⚠️ Robot State Publisher is already running, skipping..."))

    # 3. เช็ค ROBOTEQ DRIVER จริง (Mobile Base)
    if not is_process_running("roboteq_ros2_node"):
        launch_items.append(
            Node(
                package='roboteq_ros2_driver',
                executable='roboteq_ros2_node',
                name='roboteq_driver',
                parameters=[{
                    'port': '/dev/ttyUSB0', # ปรับให้ตรงกับพอร์ตจริง
                    'baudrate': 115200,
                    'channels': 2
                }],
                output='screen'
            )
        )
    else:
        launch_items.append(LogInfo(msg="⚠️ Roboteq Driver is already running, skipping..."))

    # 4. เช็ค ACTION BRIDGE
    if not is_process_running("action_bridge.py"):
        launch_items.append(
            Node(
                package='facobot_description',
                executable=os.path.join(scripts_dir, 'action_bridge.py'),
                name='action_bridge',
                output='screen'
            )
        )
    else:
        launch_items.append(LogInfo(msg="⚠️ Action Bridge is already running, skipping..."))

    # 5. เช็ค RVIZ2 (ถ้ามีคนเปิดจอทิ้งไว้แล้วไม่ต้องเปิดซ้ำ)
    if not is_process_running("rviz2"):
        launch_items.append(
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        )
    else:
        launch_items.append(LogInfo(msg="⚠️ Rviz2 is already running, skipping..."))

    return LaunchDescription(launch_items)