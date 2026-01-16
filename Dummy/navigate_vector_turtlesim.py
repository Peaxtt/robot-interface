#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from amr_interfaces.action import NavigateVector
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose # สำหรับรับค่าจากเต่า
import math
import time

class VectorNavigatorTurtlesim(Node):
    def __init__(self):
        super().__init__('vector_navigator_turtlesim')

        # --- Settings (Gain และ Tolerance ตามต้นฉบับ) ---
        self.k_v = 0.8  # Gain Linear (ปรับเพิ่มให้เต่าวิ่งไวขึ้นนิดนึง)
        self.k_w = 4.0  # Gain Angular
        self.tol_xy = 0.05 
        self.tol_th = 0.02 

        # --- State ---
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # --- Subscribers & Publishers ---
        # เปลี่ยนเป็น Topic ของ Turtlesim
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # --- Action Server ---
        self._action_server = ActionServer(
            self,
            NavigateVector,
            'navigate_vector',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("🐢 Turtlesim Vector Navigator READY!")

    def pose_callback(self, msg):
        # เต่าส่ง x, y, theta (radians) มาให้เลย ไม่ต้องแกะ Quaternion
        self.current_pose['x'] = msg.x
        self.current_pose['y'] = msg.y
        self.current_pose['theta'] = msg.theta

    def goal_callback(self, goal_request):
        self.get_logger().info('📥 Received Goal Request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('🛑 Received Cancel Request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('🚀 Executing Navigation Phases...')
        
        goal = goal_handle.request
        start_pose = self.current_pose.copy()
        feedback_msg = NavigateVector.Feedback()
        result = NavigateVector.Result()
        
        # คำนวณพิกัดเป้าหมายแบบสัมพัทธ์ (Relative Displacement)
        target_dist = math.sqrt(goal.x**2 + goal.y**2)
        target_angle_rel = math.atan2(goal.y, goal.x) # มุม vector เทียบกับหน้าหุ่นตอนเริ่ม
        
        # คำนวณมุม Global ที่ต้องหันไป
        global_target_vector_yaw = self.normalize_angle(start_pose['theta'] + target_angle_rel)
        global_final_yaw = self.normalize_angle(start_pose['theta'] + goal.theta)

        # --- PHASE 1: ALIGN TO VECTOR (หันหน้าหาเป้า) ---
        if target_dist > 0.05:
            self.get_logger().info("Phase 1: Aligning to Target Point")
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    goal_handle.canceled()
                    return NavigateVector.Result(success=False, message="Canceled")

                err_yaw = self.normalize_angle(global_target_vector_yaw - self.current_pose['theta'])
                
                feedback_msg.current_state = "ALIGN_VECTOR"
                feedback_msg.angle_remaining = err_yaw
                goal_handle.publish_feedback(feedback_msg)

                if abs(err_yaw) < self.tol_th:
                    self.stop_robot()
                    break 

                cmd = Twist()
                cmd.angular.z = max(min(self.k_w * err_yaw, 2.0), -2.0)
                self.pub_cmd.publish(cmd)
                time.sleep(0.05)

        # --- PHASE 2: MOVE LINEAR (เดินหน้า) ---
        self.get_logger().info("Phase 2: Moving Linear to Target")
        start_move_pose = self.current_pose.copy()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return NavigateVector.Result(success=False, message="Canceled")

            dx = self.current_pose['x'] - start_move_pose['x']
            dy = self.current_pose['y'] - start_move_pose['y']
            dist_traveled = math.sqrt(dx**2 + dy**2)
            err_dist = target_dist - dist_traveled
            
            # เลี้ยงหน้าให้ตรงตลอดทาง
            err_yaw_maintain = self.normalize_angle(global_target_vector_yaw - self.current_pose['theta'])

            feedback_msg.current_state = "MOVE_LINEAR"
            feedback_msg.distance_remaining = err_dist
            goal_handle.publish_feedback(feedback_msg)

            if err_dist < self.tol_xy:
                self.stop_robot()
                break

            cmd = Twist()
            cmd.linear.x = max(min(self.k_v * err_dist, 1.0), 0.1)
            cmd.angular.z = self.k_w * err_yaw_maintain 
            self.pub_cmd.publish(cmd)
            time.sleep(0.05)

        # --- PHASE 3: ALIGN FINAL (หันมุมจบ) ---
        self.get_logger().info("Phase 3: Setting Final Orientation")
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return NavigateVector.Result(success=False, message="Canceled")

            err_yaw = self.normalize_angle(global_final_yaw - self.current_pose['theta'])

            feedback_msg.current_state = "ALIGN_FINAL"
            feedback_msg.angle_remaining = err_yaw
            goal_handle.publish_feedback(feedback_msg)

            if abs(err_yaw) < self.tol_th:
                break

            cmd = Twist()
            cmd.angular.z = max(min(self.k_w * err_yaw, 2.0), -2.0)
            self.pub_cmd.publish(cmd)
            time.sleep(0.05)

        # --- FINISH ---
        self.stop_robot()
        goal_handle.succeed()
        result.success = True
        result.message = "Navigation Complete!"
        self.get_logger().info("🏁 Target Reached Successfully")
        return result

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = VectorNavigatorTurtlesim()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()