#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, UInt32, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger 

from amr_interfaces.msg import ErrorFlags
from amr_interfaces.action import NavigateVector, ExecuteQR

try:
    from ros2_modbus_driver.action import KincoMoving, MoonsMoving, PiggybackHoming
    DRIVER_AVAILABLE = True
except ImportError:
    DRIVER_AVAILABLE = False
    print("⚠️ WARN: ros2_modbus_driver not found!")

import json
import time
import math
import subprocess 

class ActionBridge(Node):
    def __init__(self):
        super().__init__('web_action_bridge')
        
        # ... Config ...
        self.LIFT_POSITIONS = { 0: 103000, 1: 365000, 2: 628000, 3: 890000 }
        self.SPD_LIFT = 600000 
        self.TURN_POSITIONS = { 0: 0, 1: 166300, 2: 330300 }
        self.SPD_TURN = 600000
        self.SLIDE_POSITIONS = { 0: 0, 1: 235000 }
        self.SPD_SLIDE = 400000
        self.HOOK_POSITIONS = { 0: 0, 1: 25000 }
        self.SPD_HOOK = 600
        self.ACTION_TIMEOUT = 60.0 

        self.cb_group_web = ReentrantCallbackGroup()
        self.cb_group_action = ReentrantCallbackGroup()

        self.create_subscription(String, '/web_command_gateway', self.web_cmd_callback, 10, callback_group=self.cb_group_web)
        self.status_pub = self.create_publisher(String, '/web_full_status', 10)
        
        self.nav_client = ActionClient(self, NavigateVector, 'navigate_vector', callback_group=self.cb_group_action)
        self.qr_client = ActionClient(self, ExecuteQR, 'execute_qr_command', callback_group=self.cb_group_action)
        self.lift_power_off_client = self.create_client(Trigger, '/modbus_device_controller/kinco_1/power_off_motor', callback_group=self.cb_group_action)

        self.piggy_clients = {}
        self.homing_client = None
        if DRIVER_AVAILABLE:
            self.setup_piggyback_clients()

        self._nav_handle = None
        self._qr_handle = None
        self._homing_handle = None
        self._piggy_handles = {}
        self._pending_hooks = 0
        self._action_timer = None
        self._lift_safety_timer = None 
        
        self.robot_state = {
            "battery": 100, 
            "position": {"x": 0.0, "y": 0.0, "th": 0.0},
            "qr_id": "WAIT",
            "active_action": "IDLE",
            "feedback_msg": "System Ready",
            "piggyback": { "lift_pos": 0, "turn_pos": 0, "slide_pos": 0, "hook_4": 0, "hook_5": 0 },
            "hardware": {"ff": 0, "fs": 0, "fm1": 0, "fm2": 0},
            "system_health": {"bringup": False, "modbus": False}
        }

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odom_qr', self.odom_qr_cb, 10) 
        self.create_subscription(String, '/qr_id', self.qr_id_cb, 10)
        self.create_subscription(ErrorFlags, '/motor_error_flags', self.flag_cb, 10)
        
        self.create_subscription(UInt32, '/modbus_driver_S0/handler/kinco_1/get_actual_pos', lambda m: self.update_piggy('lift_pos', m), 10)
        self.create_subscription(UInt32, '/modbus_driver_S1/handler/kinco_2/get_actual_pos', lambda m: self.update_piggy('turn_pos', m), 10)
        self.create_subscription(UInt32, '/modbus_driver_S1/handler/kinco_3/get_actual_pos', lambda m: self.update_piggy('slide_pos', m), 10)
        self.create_subscription(UInt32, '/modbus_driver_S1/handler/moons_4/get_actual_pos', lambda m: self.update_piggy('hook_4', m), 10)
        self.create_subscription(UInt32, '/modbus_driver_S1/handler/moons_5/get_actual_pos', lambda m: self.update_piggy('hook_5', m), 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ปรับความถี่การส่งข้อมูลเป็น 10Hz (0.1s)
        self.create_timer(0.1, self.publish_web_status, callback_group=self.cb_group_web)
        self.create_timer(1.0, self.check_node_health, callback_group=self.cb_group_web) 

        self.get_logger().info('✅ Action Bridge: STARTED (Global Sanitizer Mode)')

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
        self.homing_client = create_cli(PiggybackHoming, '/piggyback/home_all')

    def check_node_health(self):
        bringup_live = self.is_process_running("system_bringup.launch.py")
        modbus_live = self.is_process_running("modbus_node.launch.py")
        self.robot_state["system_health"]["bringup"] = bringup_live
        self.robot_state["system_health"]["modbus"] = modbus_live

    def is_process_running(self, process_name):
        try:
            return subprocess.call(["pgrep", "-f", process_name], stdout=subprocess.DEVNULL) == 0
        except:
            return False

    def update_piggy(self, key, msg):
        val = msg.data
        if val > 2147483647: val -= 4294967296
        self.robot_state["piggyback"][key] = val

    def odom_cb(self, msg): pass 

    def odom_qr_cb(self, msg):
        try:
            # คำนวณแบบดิบๆ ไปเลย (ถ้า NaN เดี๋ยวไปกรองตอนจบทีเดียว)
            self.robot_state["position"]["x"] = round(msg.pose.pose.position.x, 3)
            self.robot_state["position"]["y"] = round(msg.pose.pose.position.y, 3)
            
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_state["position"]["th"] = round(math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi, 1)
        except:
            pass 

    def qr_id_cb(self, msg): 
        # แปลงเป็น String เสมอ
        self.robot_state["qr_id"] = str(msg.data)

    def flag_cb(self, msg): 
        self.robot_state["hardware"]["ff"] = msg.ff
        self.robot_state["hardware"]["fm1"] = msg.fm1
        self.robot_state["hardware"]["fm2"] = msg.fm2

    def web_cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('stop') == True:
                self.force_stop()
                return

            cmd_type = data.get('type')
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
                goal.command_type = 3
                goal.iterate = 1
                goal.data_collect = False 
                raw_path = data.get('path', [])
                goal.target_qr_ids = [str(x) for x in raw_path]
                self.send_qr_goal(goal, "MISSION")

            elif cmd_type == 'PIGGYBACK_MANUAL':
                if not DRIVER_AVAILABLE: return
                self.handle_piggyback_manual(data)

            elif cmd_type == 'PIGGYBACK_HOME':
                if not DRIVER_AVAILABLE: return
                self.execute_homing()

            elif cmd_type == 'SYSTEM_RESET':
                self.execute_system_reset()

        except Exception as e:
            self.get_logger().error(f'❌ JSON Error: {e}')

    def handle_piggyback_manual(self, data):
        # ... (เหมือนเดิม) ...
        comp = int(data.get('component', -1))
        val = int(data.get('value', 0))
        current_slide = self.robot_state["piggyback"]["slide_pos"]
        current_hook = self.robot_state["piggyback"]["hook_4"]
        SLIDE_EXTENDED_THRESHOLD = 5000 
        HOOK_LOCKED_THRESHOLD = 5000     

        hw = self.robot_state["hardware"]
        if hw["ff"] != 0 or hw["fs"] != 0 or hw["fm1"] != 0 or hw["fm2"] != 0:
             self.robot_state["feedback_msg"] = "HARDWARE ERROR"
             return

        if (comp == 0 or comp == 1):
            if current_slide > SLIDE_EXTENDED_THRESHOLD:
                self.robot_state["feedback_msg"] = "SAFETY: SLIDE EXTENDED"
                return
        if comp == 2 and val == 1: 
            if current_hook > HOOK_LOCKED_THRESHOLD:
                self.robot_state["feedback_msg"] = "SAFETY: GRIPPER LOCKED"
                return

        if comp == 0: 
            target = self.LIFT_POSITIONS.get(val, 103000)
            self.send_kinco_goal('k1', target, self.SPD_LIFT, f"LIFTING TO F{val+1}", "LIFTING", is_lift=True)
        elif comp == 1: 
            target = self.TURN_POSITIONS.get(val, 0)
            labels = {0: "RIGHT", 1: "LEFT", 2: "BACK"}
            self.send_kinco_goal('k2', target, self.SPD_TURN, f"TURNING {labels.get(val,'')}", "TURNING")
        elif comp == 2: 
            target = self.SLIDE_POSITIONS.get(val, 0)
            labels = {0: "IN", 1: "OUT"}
            self.send_kinco_goal('k3', target, self.SPD_SLIDE, f"SLIDING {labels.get(val,'')}", "SLIDING")
        elif comp == 3: 
            target = self.HOOK_POSITIONS.get(val, 0)
            self.send_hook_pair(target)

    def execute_homing(self):
        if not self.homing_client.wait_for_server(timeout_sec=1.0): return
        self.cancel_all_goals()
        goal = PiggybackHoming.Goal()
        self.robot_state["active_action"] = "HOMING" 
        self.robot_state["feedback_msg"] = "STARTING HOME ALL..."
        future = self.homing_client.send_goal_async(goal, feedback_callback=self.homing_feedback)
        future.add_done_callback(lambda f: self.handle_goal_response(f, "HOMING"))
        self.start_action_timeout("HOMING")

    def homing_feedback(self, feedback_msg):
        msg = feedback_msg.feedback
        self.robot_state["feedback_msg"] = f"{msg.state}"

    def send_kinco_goal(self, client_key, pos, vel, status_msg, action_tag="PIGGYBACK", is_lift=False):
        client = self.piggy_clients.get(client_key)
        if not client or not client.wait_for_server(timeout_sec=0.5): return
        self.robot_state["active_action"] = action_tag
        self.robot_state["feedback_msg"] = status_msg
        goal = KincoMoving.Goal()
        goal.target_position = int(pos)
        goal.velocity = int(vel)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.kinco_goal_accepted(f, client_key, is_lift))
        self.start_action_timeout(action_tag)

    def kinco_goal_accepted(self, future, client_key, is_lift):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.robot_state["active_action"] = "IDLE"
                self.robot_state["feedback_msg"] = "GOAL REJECTED"
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self.kinco_result_done(f, client_key, is_lift))
        except Exception as e:
            self.get_logger().error(f"Kinco Goal Error: {e}")
            self.robot_state["active_action"] = "IDLE"

    def kinco_result_done(self, future, client_key, is_lift):
        self.cancel_action_timeout()
        if is_lift:
             self.robot_state["feedback_msg"] = "LIFT HOLDING..."
             self._lift_safety_timer = self.create_timer(1.0, self.trigger_lift_power_off)
        else:
             self.robot_state["active_action"] = "IDLE"
             self.robot_state["feedback_msg"] = "DONE"

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
            if self._pending_hooks <= 0: 
                self.robot_state["active_action"] = "IDLE"
            return
        goal = MoonsMoving.Goal()
        goal.target_position = int(pos)
        goal.velocity = self.SPD_HOOK
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.moons_goal_accepted(f, key))

    def moons_goal_accepted(self, future, key):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._pending_hooks -= 1
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self.moons_result_done(f, key))
        except: pass

    def moons_result_done(self, future, key):
        self._pending_hooks -= 1
        if self._pending_hooks <= 0:
            self.robot_state["active_action"] = "IDLE"
            self.robot_state["feedback_msg"] = "HOOK DONE"
            self.cancel_action_timeout()

    def trigger_lift_power_off(self):
        if self._lift_safety_timer: self._lift_safety_timer.cancel()
        req = Trigger.Request()
        future = self.lift_power_off_client.call_async(req)
        future.add_done_callback(self.lift_off_done)

    def lift_off_done(self, future):
        self.robot_state["active_action"] = "IDLE"
        self.robot_state["feedback_msg"] = "LIFT LOCKED (OFF)"
        self.cancel_action_timeout()

    def handle_goal_response(self, future, action_name):
        try:
            handle = future.result()
            if not handle.accepted:
                self.robot_state["active_action"] = "IDLE"
                self.robot_state["feedback_msg"] = "GOAL REJECTED"
                return
            if action_name == "NAVIGATION": self._nav_handle = handle
            if action_name == "QR": self._qr_handle = handle
            if action_name == "HOMING": self._homing_handle = handle
            handle.get_result_async().add_done_callback(self.action_done)
        except: pass

    def action_done(self, future):
        self.robot_state["active_action"] = "IDLE"
        self.robot_state["feedback_msg"] = "COMPLETED"
        self.cancel_action_timeout()

    def send_nav_goal(self, goal):
        # ... (เหมือนเดิม) ...
        if not self.nav_client.wait_for_server(timeout_sec=1.0): return
        self.robot_state["active_action"] = "NAVIGATING"
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.handle_goal_response(f, "NAVIGATION"))
        self.start_action_timeout('NAVIGATION')

    def send_qr_goal(self, goal, mode_name):
        # ... (เหมือนเดิม) ...
        if not self.qr_client.wait_for_server(timeout_sec=1.0): return
        self.robot_state["active_action"] = mode_name
        future = self.qr_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.handle_goal_response(f, "QR"))
        self.start_action_timeout(mode_name)

    def execute_system_reset(self):
        self.get_logger().warn("🔄 SYSTEM RESET REQUESTED")
        self.robot_state["feedback_msg"] = "RESETTING SYSTEM..."
        self.publish_web_status()
        import subprocess
        try:
             subprocess.Popen(
                 ["/home/admin/reset_system.sh"], 
                 shell=True,
                 stdin=None, 
                 stdout=None, 
                 stderr=None, 
                 close_fds=True,
                 start_new_session=True 
             )
        except Exception as e:
             self.get_logger().error(f"Reset Script Failed: {e}")

    def force_stop(self):
        self.cancel_all_goals()
        stop_cmd = Twist()
        for _ in range(5): self.cmd_vel_pub.publish(stop_cmd)
        self.robot_state["feedback_msg"] = "ESTOP ACTIVE"
        self.robot_state["active_action"] = "IDLE" 
        self.get_logger().warn("🚨 FORCE STOP EXECUTED")

    def cancel_all_goals(self):
        if self._nav_handle:
            try: self._nav_handle.cancel_goal_async()
            except: pass
            self._nav_handle = None
        if self._qr_handle:
            try: self._qr_handle.cancel_goal_async()
            except: pass
            self._qr_handle = None
        if self._homing_handle:
            try: self._homing_handle.cancel_goal_async()
            except: pass
            self._homing_handle = None
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

    # 🔥🔥 MAGIC FUNCTION: ฟังก์ชันทำความสะอาดข้อมูล (Recursive) 🔥🔥
    # มันจะเจาะเข้าไปทุกชั้นของ Dictionary ถ้าเจอ NaN/Inf จะเปลี่ยนเป็น 0.0 ทันที
    def sanitize_for_json(self, obj):
        if isinstance(obj, float):
            if math.isnan(obj) or math.isinf(obj):
                return 0.0
            return obj
        elif isinstance(obj, dict):
            return {k: self.sanitize_for_json(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self.sanitize_for_json(v) for v in obj]
        return obj

    # 🛡️🛡️🛡️ จุดดักจับสุดท้าย (The Final Gate) 🛡️🛡️🛡️
    def publish_web_status(self):
        self.robot_state["last_update"] = time.time()
        try:
            # 1. ทำความสะอาดข้อมูลทั้งก้อนก่อนส่ง
            clean_state = self.sanitize_for_json(self.robot_state)
            
            # 2. แปลงเป็น JSON (ตอนนี้ปลอดภัย 100% แล้ว)
            msg = String()
            msg.data = json.dumps(clean_state)
            self.status_pub.publish(msg)
        except Exception as e:
            # ถ้ายังพังอีก (ซึ่งยากมาก) ให้ Log ไว้ แต่ห้ามตาย
            self.get_logger().error(f"JSON Publish Failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ActionBridge()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()