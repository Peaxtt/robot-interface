#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import Interface
from amr_interfaces.msg import MotorAmp, ErrorFlags
from amr_interfaces.action import NavigateVector, ExecuteQR

# Check Drivers
try:
    from ros2_modbus_driver.action import KincoMoving, MoonsMoving
    DRIVER_AVAILABLE = True
except ImportError:
    DRIVER_AVAILABLE = False
    print("⚠️ WARN: ros2_modbus_driver not found! Piggyback will not work.")

import json
import time
import math

class ActionBridge(Node):
    def __init__(self):
        super().__init__('web_action_bridge')
        
        # ==========================================
        # 🔧 PARAMETER SETUP (Load from YAML)
        # ==========================================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ratio_lift', 1000000.0),
                ('ratio_slide', 1000000.0),
                ('ratio_turn', 2000.0),
                ('speed_lift', 300000),
                ('speed_turn', 200000),
                ('speed_slide', 300000),
                ('speed_hook', 600),
                # Config สำหรับ M4
                ('hook_m4_lock', 25000),
                ('hook_m4_unlock', 0),
                # Config สำหรับ M5 (เผื่อกรณีติดกลับด้าน)
                ('hook_m5_lock', 0), 
                ('hook_m5_unlock', 25000),
                ('action_timeout', 30.0)
            ]
        )

        # ดึงค่า Config (ถ้าไม่มีใน yaml จะใช้ค่า Default ที่ตั้งไว้ข้างบน)
        self.LIFT_RATIO = self.get_parameter('ratio_lift').value
        self.SLIDE_RATIO = self.get_parameter('ratio_slide').value
        self.TURN_RATIO = self.get_parameter('ratio_turn').value
        
        self.SPD_LIFT = self.get_parameter('speed_lift').value
        self.SPD_TURN = self.get_parameter('speed_turn').value
        self.SPD_SLIDE = self.get_parameter('speed_slide').value
        self.SPD_HOOK = self.get_parameter('speed_hook').value

        self.HOOK_LOCK_M4 = self.get_parameter('hook_m4_lock').value
        self.HOOK_UNLOCK_M4 = self.get_parameter('hook_m4_unlock').value
        self.HOOK_LOCK_M5 = self.get_parameter('hook_m5_lock').value
        self.HOOK_UNLOCK_M5 = self.get_parameter('hook_m5_unlock').value
        
        self.ACTION_TIMEOUT = self.get_parameter('action_timeout').value

        self.get_logger().info(f"⚙️ Config Loaded. Timeout: {self.ACTION_TIMEOUT}s")

        # 1. THREADING SETUP
        self.cb_group_web = ReentrantCallbackGroup()
        self.cb_group_action = ReentrantCallbackGroup()

        # 2. COMMUNICATION GATEWAY
        # รับคำสั่งจาก Web
        self.create_subscription(String, '/web_command_gateway', 
                                 self.web_cmd_callback, 10, 
                                 callback_group=self.cb_group_web)
        # ส่งสถานะกลับ Web
        self.status_pub = self.create_publisher(String, '/web_full_status', 10)
        
        # 3. ACTION CLIENTS (Mobile Base)
        self.nav_client = ActionClient(self, NavigateVector, 'navigate_vector', callback_group=self.cb_group_action)
        self.qr_client = ActionClient(self, ExecuteQR, 'execute_qr_command', callback_group=self.cb_group_action)
        
        # 4. ACTION CLIENTS (Piggyback)
        self.piggy_clients = {}
        if DRIVER_AVAILABLE:
            self.setup_piggyback_clients()

        # Goal Handles & State
        self._nav_handle = None
        self._qr_handle = None
        self._piggy_handles = {}
        
        # Hook Synchronization
        self._pending_hooks = 0
        self._hook_results = {'m4': None, 'm5': None}
        
        # Timeout Timer
        self._action_timer = None
        
        self.robot_state = {
            "battery": 100, 
            "voltage": 24.0,
            "position": {"x": 0.0, "y": 0.0, "th": 0.0},
            "hardware": {"ff": 0, "fs": 0, "fm1": 0, "fm2": 0},
            "active_action": "IDLE",
            "feedback_msg": "System Ready",
            "connection": "ONLINE",
            "last_update": time.time()
        }

        # Subscribers (Odom & HW Flags)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(ErrorFlags, '/motor_error_flags', self.flag_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Loop ส่งสถานะกลับเว็บ (10Hz)
        self.create_timer(0.1, self.publish_web_status, callback_group=self.cb_group_web)
        
        self.get_logger().info('✅ Action Bridge: STARTED')

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

    # ============================================
    # 📡 SENSOR CALLBACKS
    # ============================================
    def odom_cb(self, msg):
        self.robot_state["position"]["x"] = round(msg.pose.pose.position.x, 3)
        self.robot_state["position"]["y"] = round(msg.pose.pose.position.y, 3)
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta_rad = math.atan2(siny_cosp, cosy_cosp)
        self.robot_state["position"]["th"] = round(theta_rad * 180.0 / math.pi, 1)

    def flag_cb(self, msg): 
        self.robot_state["hardware"]["ff"] = msg.ff
        self.robot_state["hardware"]["fm1"] = msg.fm1
        self.robot_state["hardware"]["fm2"] = msg.fm2

    # ============================================
    # 🎮 WEB COMMAND HANDLER
    # ============================================
    def web_cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            cmd_type = data.get('type')
            
            # Emergency Stop
            if data.get('stop') == True:
                self.force_stop()
                return

            # --- NAVIGATE VECTOR (Manual เดินตามพิกัด) ---
            if cmd_type == 'NAVIGATE_VECTOR':
                self.cancel_all_goals()
                goal = NavigateVector.Goal()
                goal.x = float(data.get('x', 0.0))
                goal.y = float(data.get('y', 0.0))
                goal.theta = float(data.get('theta', 0.0))
                self.send_nav_goal(goal)

            # --- EXECUTE MISSION (Auto QR) ---
            elif cmd_type == 'EXECUTE_MISSION':
                self.cancel_all_goals()
                goal = ExecuteQR.Goal()
                goal.command_type = 3  # Mode 3: Approach + Docking
                goal.target_qr_ids = [int(x) for x in data.get('path', [])]
                
                # [FIX CRITICAL] รับค่ามาแสดงผล แต่ไม่ส่งเข้า Goal เพราะ Action ไม่มี field นี้
                final_heading = data.get('final_heading', 0)
                self.get_logger().info(f"📥 Mission: Path={goal.target_qr_ids}, Heading={final_heading} (Ignored)")
                
                self.send_qr_goal(goal, "MISSION")

            # --- ALIGN DOCK (สั่ง Docking จุดเดียว) ---
            elif cmd_type == 'ALIGN_DOCK':
                self.cancel_all_goals()
                goal = ExecuteQR.Goal()
                goal.command_type = 1 
                target_id = int(data.get('qr_id', 0))
                goal.target_qr_ids = [target_id] if target_id > 0 else []
                self.send_qr_goal(goal, "DOCKING")

            # --- PIGGYBACK MANUAL ---
            elif cmd_type == 'PIGGYBACK_MANUAL':
                if not DRIVER_AVAILABLE:
                    self.robot_state["feedback_msg"] = "DRIVER MISSING"
                    return
                self.handle_piggyback_manual(data)

        except Exception as e:
            self.get_logger().error(f'❌ JSON Error: {e}')
            self.robot_state["feedback_msg"] = f"CMD ERROR"

    def handle_piggyback_manual(self, data):
        comp = int(data.get('component', -1))
        val = float(data.get('value', 0.0))
        
        if comp == 0:  # LIFT
            pulse = int(val * self.LIFT_RATIO)
            self.send_kinco_goal('k1', pulse, self.SPD_LIFT, "LIFTING")
        elif comp == 1:  # TURN
            pulse = int(val * self.TURN_RATIO)
            self.send_kinco_goal('k2', pulse, self.SPD_TURN, "TURNING")
        elif comp == 2:  # SLIDE
            pulse = int(val * self.SLIDE_RATIO)
            self.send_kinco_goal('k3', pulse, self.SPD_SLIDE, "SLIDING")
        elif comp == 3:  # HOOK
            is_lock = (val > 0.5)
            self.send_hook_pair(is_lock)

    # ============================================
    # ACTION SENDERS (Mobile Base)
    # ============================================
    def send_nav_goal(self, goal):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.robot_state["feedback_msg"] = "NAV SERVER OFFLINE"
            return
        
        self.robot_state["active_action"] = "NAVIGATING"
        self.robot_state["feedback_msg"] = "Starting navigation..."
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.generic_feedback)
        future.add_done_callback(lambda f: self.handle_goal_response(f, "NAVIGATION"))
        self.start_action_timeout('NAVIGATION')

    def send_qr_goal(self, goal, mode_name):
        if not self.qr_client.wait_for_server(timeout_sec=1.0):
            self.robot_state["feedback_msg"] = "QR SERVER OFFLINE"
            return
            
        self.robot_state["active_action"] = mode_name
        self.robot_state["feedback_msg"] = f"Starting {mode_name}..."
        future = self.qr_client.send_goal_async(goal, feedback_callback=self.generic_feedback)
        future.add_done_callback(lambda f: self.handle_goal_response(f, "QR"))
        self.start_action_timeout(mode_name)

    def generic_feedback(self, feedback_msg):
        if hasattr(feedback_msg.feedback, 'current_state'):
            self.robot_state["feedback_msg"] = str(feedback_msg.feedback.current_state)

    def handle_goal_response(self, future, action_name):
        try:
            handle = future.result()
            if not handle.accepted:
                self.robot_state["active_action"] = "IDLE"
                self.robot_state["feedback_msg"] = f"{action_name} REJECTED"
                self.cancel_action_timeout()
                return

            # Store handle
            if action_name == "NAVIGATION": self._nav_handle = handle
            else: self._qr_handle = handle
            
            handle.get_result_async().add_done_callback(self.action_done)
        except Exception as e:
            self.get_logger().error(f'{action_name} response error: {e}')

    def action_done(self, future):
        try:
            result = future.result()
            self.robot_state["active_action"] = "IDLE"
            if result.result.success:
                self.robot_state["feedback_msg"] = "COMPLETED"
            else:
                self.robot_state["feedback_msg"] = f"FAILED: {result.result.message}"
        except Exception:
            self.robot_state["feedback_msg"] = "RESULT ERROR"
        finally:
            self.cancel_action_timeout()

    # ============================================
    # ACTION SENDERS (Piggyback)
    # ============================================
    def send_kinco_goal(self, client_key, pos, vel, status_msg):
        client = self.piggy_clients.get(client_key)
        if not client or not client.wait_for_server(timeout_sec=0.5):
            self.robot_state["feedback_msg"] = f"HW OFFLINE: {client_key.upper()}"
            return

        self.robot_state["active_action"] = "PIGGYBACK"
        self.robot_state["feedback_msg"] = status_msg
        
        goal = KincoMoving.Goal()
        goal.target_position = pos
        goal.velocity = vel
        
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.piggy_goal_accepted(f, client_key))
        self.start_action_timeout('PIGGYBACK')

    def piggy_goal_accepted(self, future, motor_key):
        try:
            handle = future.result()
            if not handle.accepted:
                self.robot_state["active_action"] = "IDLE"
                self.robot_state["feedback_msg"] = f"{motor_key.upper()} REJECTED"
                return
            
            self._piggy_handles[motor_key] = handle
            handle.get_result_async().add_done_callback(lambda f: self.piggy_result_done(f, motor_key))
        except: pass

    def piggy_result_done(self, future, motor_key):
        self.robot_state["active_action"] = "IDLE"
        self.robot_state["feedback_msg"] = "DONE"
        if motor_key in self._piggy_handles:
            del self._piggy_handles[motor_key]
        self.cancel_action_timeout()

    def send_hook_pair(self, is_lock):
        # ใช้ค่าจาก Params (รองรับกรณีมอเตอร์ติดกลับด้าน)
        m4_target = self.HOOK_LOCK_M4 if is_lock else self.HOOK_UNLOCK_M4
        m5_target = self.HOOK_LOCK_M5 if is_lock else self.HOOK_UNLOCK_M5
        
        self.robot_state["active_action"] = "GRIPPER"
        self.robot_state["feedback_msg"] = "LOCKING..." if is_lock else "UNLOCKING..."
        
        self._pending_hooks = 2
        self._hook_results = {'m4': None, 'm5': None}
        
        self.send_moons_with_tracking('m4', m4_target, self.SPD_HOOK)
        self.send_moons_with_tracking('m5', m5_target, self.SPD_HOOK)
        self.start_action_timeout('GRIPPER')

    def send_moons_with_tracking(self, key, pos, vel):
        client = self.piggy_clients.get(key)
        if not client: return
        
        if not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(f'{key.upper()} offline')
            self._pending_hooks -= 1
            if self._pending_hooks <= 0:
                self.check_hook_completion()
            return

        goal = MoonsMoving.Goal()
        goal.target_position = pos
        goal.velocity = vel
        
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.hook_goal_accepted(f, key))

    def hook_goal_accepted(self, future, key):
        try:
            handle = future.result()
            if handle.accepted:
                handle.get_result_async().add_done_callback(lambda f: self.hook_result_done(f, key))
            else:
                self._pending_hooks -= 1
                self._hook_results[key] = False
                self.check_hook_completion()
        except: 
            self._pending_hooks -= 1
            self.check_hook_completion()

    def hook_result_done(self, future, motor_key):
        try:
            result = future.result()
            success = True
            if hasattr(result, 'result') and hasattr(result.result, 'success'):
                success = result.result.success
            self._hook_results[motor_key] = success
        except:
            self._hook_results[motor_key] = False
        finally:
            self._pending_hooks -= 1
            self.check_hook_completion()

    def check_hook_completion(self):
        if self._pending_hooks <= 0:
            m4_ok = self._hook_results.get('m4', False)
            m5_ok = self._hook_results.get('m5', False)
            
            self.robot_state["active_action"] = "IDLE"
            
            if m4_ok and m5_ok:
                self.robot_state["feedback_msg"] = "GRIPPER DONE"
            elif m4_ok or m5_ok:
                self.robot_state["feedback_msg"] = "PARTIAL SUCCESS"
            else:
                self.robot_state["feedback_msg"] = "GRIPPER FAILED"
            
            self.cancel_action_timeout()

    # ============================================
    # UTILS
    # ============================================
    def force_stop(self):
        self.get_logger().warn("🚨 EMERGENCY STOP!")
        self.cancel_all_goals()
        # ส่ง 0 ไปที่ล้อ
        stop_cmd = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_cmd)
        
        # Cancel Piggyback
        for h in self._piggy_handles.values():
            try: h.cancel_goal_async()
            except: pass
        self._piggy_handles.clear()
        
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
        self._action_timer = self.create_timer(self.ACTION_TIMEOUT, lambda: self.timeout_handler(name), callback_group=self.cb_group_action)

    def timeout_handler(self, name):
        self.get_logger().warn(f"⏱️ Timeout: {name}")
        self.robot_state["active_action"] = "TIMEOUT"
        self.robot_state["feedback_msg"] = "TIMEOUT"
        self.cancel_all_goals()
        self.cancel_action_timeout()

    def cancel_action_timeout(self):
        if self._action_timer:
            self._action_timer.cancel()
            self._action_timer = None

    def publish_web_status(self):
        self.robot_state["last_update"] = time.time()
        msg = String()
        msg.data = json.dumps(self.robot_state)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActionBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()