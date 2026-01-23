#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import Interface
from amr_interfaces.msg import ErrorFlags
from amr_interfaces.action import NavigateVector, ExecuteQR

# Check Drivers
try:
    from ros2_modbus_driver.action import KincoMoving, MoonsMoving
    DRIVER_AVAILABLE = True
except ImportError:
    DRIVER_AVAILABLE = False
    print("⚠️ WARN: ros2_modbus_driver not found!")

import json
import time
import math

class ActionBridge(Node):
    def __init__(self):
        super().__init__('web_action_bridge')
        
        # ==========================================
        # 🎯 FIX POSITIONS (LOOKUP TABLE)
        # ==========================================
        # Lift Floors: 1, 2, 3, 4 (Index 0-3)
        self.LIFT_POSITIONS = {
            0: 103000, 1: 365000, 2: 628000, 3: 890000
        }
        self.SPD_LIFT = 600000

        # Turntable: Right, Left, Back (Index 0-2)
        self.TURN_POSITIONS = {
            0: 0,       # Right (0 deg)
            1: 166300,  # Left  (180 deg by pulse logic)
            2: 330300   # Back  (90 deg by pulse logic - User Defined)
        }
        self.SPD_TURN = 600000

        # Slide: In, Out (Index 0-1)
        self.SLIDE_POSITIONS = {
            0: 0,       # In
            1: 235000   # Out
        }
        self.SPD_SLIDE = 400000

        # Hook: Unlock, Lock (Index 0-1)
        self.HOOK_POSITIONS = {
            0: 0,       # Unlock
            1: 25000    # Lock
        }
        self.SPD_HOOK = 600
        
        self.ACTION_TIMEOUT = 30.0

        # 1. THREADING
        self.cb_group_web = ReentrantCallbackGroup()
        self.cb_group_action = ReentrantCallbackGroup()

        # 2. COMMUNICATION
        self.create_subscription(String, '/web_command_gateway', self.web_cmd_callback, 10, callback_group=self.cb_group_web)
        self.status_pub = self.create_publisher(String, '/web_full_status', 10)
        
        # 3. CLIENTS
        self.nav_client = ActionClient(self, NavigateVector, 'navigate_vector', callback_group=self.cb_group_action)
        self.qr_client = ActionClient(self, ExecuteQR, 'execute_qr_command', callback_group=self.cb_group_action)
        
        self.piggy_clients = {}
        if DRIVER_AVAILABLE:
            self.setup_piggyback_clients()

        # State
        self._nav_handle = None
        self._qr_handle = None
        self._piggy_handles = {}
        self._pending_hooks = 0
        self._hook_results = {'m4': None, 'm5': None}
        self._action_timer = None
        
        self.robot_state = {
            "battery": 100, 
            "position": {"x": 0.0, "y": 0.0, "th": 0.0},
            "qr_id": "WAIT",
            "active_action": "IDLE",
            "feedback_msg": "System Ready",
            # Real-time Feedback (Pulse)
            "piggyback": {
                "lift_pos": 0,
                "turn_pos": 0,
                "slide_pos": 0,
                "hook_4": 0,
                "hook_5": 0
            },
            "hardware": {"ff": 0, "fs": 0, "fm1": 0, "fm2": 0}
        }

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odom_qr', self.odom_qr_cb, 10) # QR Pose Primary
        self.create_subscription(String, '/qr_id', self.qr_id_cb, 10)
        self.create_subscription(ErrorFlags, '/motor_error_flags', self.flag_cb, 10)
        
        # ⭐ FEEDBACK SUBSCRIBERS (Real Pulse)
        self.create_subscription(Int32, '/modbus_driver_S1/handler/kinco_1/get_actual_pos', lambda m: self.update_piggy('lift_pos', m), 10)
        self.create_subscription(Int32, '/modbus_driver_S1/handler/kinco_2/get_actual_pos', lambda m: self.update_piggy('turn_pos', m), 10)
        self.create_subscription(Int32, '/modbus_driver_S1/handler/kinco_3/get_actual_pos', lambda m: self.update_piggy('slide_pos', m), 10)
        self.create_subscription(Int32, '/modbus_driver_S1/handler/moons_4/get_actual_pos', lambda m: self.update_piggy('hook_4', m), 10)
        self.create_subscription(Int32, '/modbus_driver_S1/handler/moons_5/get_actual_pos', lambda m: self.update_piggy('hook_5', m), 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.publish_web_status, callback_group=self.cb_group_web)
        self.get_logger().info('✅ Action Bridge: STARTED (Fixed Pulse Mode)')

    def setup_piggyback_clients(self):
        def create_cli(cls, topic):
            return ActionClient(self, cls, topic, callback_group=self.cb_group_action)
        self.piggy_clients = {
            'k1': create_cli(KincoMoving, '/modbus_device_controller/kinco_1/moving'),
            'k2': create_cli(KincoMoving, '/modbus_device_controller/kinco_2/moving'),
            'k3': create_cli(KincoMoving, '/modbus_device_controller/kinco_3/moving'),
            'm4': create_cli(MoonsMoving, '/modbus_device_controller/moons_4/moving'),
            'm5': create_cli(MoonsMoving, '/modbus_device_controller/moons_5/moving')
        }

    # --- CALLBACKS ---
    def update_piggy(self, key, msg):
        self.robot_state["piggyback"][key] = msg.data

    def odom_cb(self, msg): pass # Ignored
    
    def odom_qr_cb(self, msg):
        # ⭐ PRIMARY: Use QR Odom
        self.robot_state["position"]["x"] = round(msg.pose.pose.position.x, 3)
        self.robot_state["position"]["y"] = round(msg.pose.pose.position.y, 3)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta_rad = math.atan2(siny_cosp, cosy_cosp)
        self.robot_state["position"]["th"] = round(theta_rad * 180.0 / math.pi, 1)

    def qr_id_cb(self, msg):
        self.robot_state["qr_id"] = msg.data
        
    def flag_cb(self, msg): 
        self.robot_state["hardware"]["ff"] = msg.ff
        self.robot_state["hardware"]["fm1"] = msg.fm1
        self.robot_state["hardware"]["fm2"] = msg.fm2

    # --- COMMAND HANDLER ---
    def web_cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            cmd_type = data.get('type')
            
            if data.get('stop') == True:
                self.force_stop()
                return

            if cmd_type == 'NAVIGATE_VECTOR':
                self.cancel_all_goals()
                goal = NavigateVector.Goal()
                goal.x = float(data.get('x', 0.0))
                goal.y = float(data.get('y', 0.0))
                goal.theta = float(data.get('theta', 0.0))
                self.send_nav_goal(goal)

            elif cmd_type == 'EXECUTE_MISSION':
                self.cancel_all_goals()
                goal = ExecuteQR.Goal()
                goal.command_type = 3 # Fixed Type 3
                raw_path = data.get('path', [])
                goal.target_qr_ids = [int(x) for x in raw_path]
                self.send_qr_goal(goal, "MISSION")

            elif cmd_type == 'PIGGYBACK_MANUAL':
                if not DRIVER_AVAILABLE: return
                self.handle_piggyback_manual(data)

        except Exception as e:
            self.get_logger().error(f'❌ JSON Error: {e}')

    def handle_piggyback_manual(self, data):
        comp = int(data.get('component', -1))
        val = int(data.get('value', 0)) # รับเป็น Index (0,1,2,3...)
        
        # ใช้ Dictionary ด้านบนแปลง Index -> Pulse ที่ถูกต้อง
        if comp == 0:  # LIFT
            target = self.LIFT_POSITIONS.get(val, 103000)
            self.send_kinco_goal('k1', target, self.SPD_LIFT, f"LIFTING TO F{val+1}")

        elif comp == 1:  # TURN
            target = self.TURN_POSITIONS.get(val, 0)
            labels = {0: "RIGHT", 1: "LEFT", 2: "BACK"}
            self.send_kinco_goal('k2', target, self.SPD_TURN, f"TURNING {labels.get(val,'')}")

        elif comp == 2:  # SLIDE
            target = self.SLIDE_POSITIONS.get(val, 0)
            labels = {0: "IN", 1: "OUT"}
            self.send_kinco_goal('k3', target, self.SPD_SLIDE, f"SLIDING {labels.get(val,'')}")

        elif comp == 3:  # HOOK
            target = self.HOOK_POSITIONS.get(val, 0)
            self.send_hook_pair(target)

    # --- ACTION SENDERS (Keep same logic) ---
    def send_nav_goal(self, goal):
        if not self.nav_client.wait_for_server(timeout_sec=1.0): return
        self.robot_state["active_action"] = "NAVIGATING"
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.handle_goal_response(f, "NAVIGATION"))
        self.start_action_timeout('NAVIGATION')

    def send_qr_goal(self, goal, mode_name):
        if not self.qr_client.wait_for_server(timeout_sec=1.0): return
        self.robot_state["active_action"] = mode_name
        future = self.qr_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.handle_goal_response(f, "QR"))
        self.start_action_timeout(mode_name)

    def send_kinco_goal(self, client_key, pos, vel, status_msg):
        client = self.piggy_clients.get(client_key)
        if not client or not client.wait_for_server(timeout_sec=0.5): return
        self.robot_state["active_action"] = "PIGGYBACK"
        self.robot_state["feedback_msg"] = status_msg
        goal = KincoMoving.Goal()
        goal.target_position = int(pos)
        goal.velocity = int(vel)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.piggy_done(f, client_key))
        self.start_action_timeout('PIGGYBACK')

    def send_hook_pair(self, target_pos):
        self.robot_state["active_action"] = "GRIPPER"
        self._pending_hooks = 2
        self.send_moons('m4', target_pos)
        self.send_moons('m5', target_pos)
        self.start_action_timeout('GRIPPER')

    def send_moons(self, key, pos):
        client = self.piggy_clients.get(key)
        if not client or not client.wait_for_server(timeout_sec=0.5):
            self._pending_hooks -= 1
            if self._pending_hooks <= 0: self.piggy_done(None, "HOOK")
            return
        goal = MoonsMoving.Goal()
        goal.target_position = int(pos)
        goal.velocity = self.SPD_HOOK
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.hook_done(f, key))

    # --- HANDLERS (Same) ---
    def handle_goal_response(self, future, action_name):
        try:
            handle = future.result()
            if not handle.accepted:
                self.robot_state["active_action"] = "IDLE"
                return
            handle.get_result_async().add_done_callback(self.action_done)
        except: pass

    def action_done(self, future):
        self.robot_state["active_action"] = "IDLE"
        self.cancel_action_timeout()

    def piggy_done(self, future, key):
        self.robot_state["active_action"] = "IDLE"
        self.robot_state["feedback_msg"] = "DONE"
        self.cancel_action_timeout()

    def hook_done(self, future, key):
        self._pending_hooks -= 1
        if self._pending_hooks <= 0:
            self.robot_state["active_action"] = "IDLE"
            self.robot_state["feedback_msg"] = "HOOK DONE"
            self.cancel_action_timeout()

    def force_stop(self):
        self.cancel_all_goals()
        for h in self._piggy_handles.values(): h.cancel_goal_async()
        stop_cmd = Twist()
        for _ in range(5): self.cmd_vel_pub.publish(stop_cmd)
        self.robot_state["feedback_msg"] = "ESTOP ACTIVE"

    def cancel_all_goals(self):
        if self._nav_handle:
            try: self._nav_handle.cancel_goal_async()
            except: pass
            self._nav_handle = None
        if self._qr_handle:
            try: self._qr_handle.cancel_goal_async()
            except: pass
            self._qr_handle = None
        self.cancel_action_timeout()

    def start_action_timeout(self, name):
        self.cancel_action_timeout()
        self._action_timer = self.create_timer(self.ACTION_TIMEOUT, lambda: self.timeout(name))
    def timeout(self, name):
        self.robot_state["active_action"] = "TIMEOUT"
        self.robot_state["feedback_msg"] = "TIMEOUT"
        self.cancel_action_timeout()
    def cancel_action_timeout(self):
        if self._action_timer: self._action_timer.cancel()

    def publish_web_status(self):
        self.robot_state["last_update"] = time.time()
        msg = String()
        msg.data = json.dumps(self.robot_state)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActionBridge()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()