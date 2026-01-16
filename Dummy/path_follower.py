import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import json
import math
import time

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        self.Kp_linear = 1.2
        self.Kp_angular = 5.0
        self.stop_distance = 0.05
        
        self.pose = None
        self.is_moving = False
        self.initial_theta = None # จำมุมตอนเริ่มรัน Path ทั้งหมด

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # รับพิกัดแบบเมตรที่จะให้เลื่อนจากจุดปัจจุบัน (Delta)
        self.path_sub = self.create_subscription(String, '/navigation/path', self.path_callback, 10)
        
        self.get_logger().info("✅ Smart Path Follower Ready (Relative Vector Mode)")

    def pose_callback(self, msg):
        self.pose = msg

    def path_callback(self, msg):
        try:
            # ข้อมูลที่ส่งมาจะเป็นรายการเวกเตอร์สัมพัทธ์ [[dx1, dy1], [dx2, dy2], ...]
            vectors = json.loads(msg.data)
            if not vectors or len(vectors) == 0:
                self.stop_robot()
                return

            self.initial_theta = self.pose.theta # จำมุมก่อนเริ่ม
            
            for dx, dy in vectors:
                self.get_logger().info(f"🚀 Moving by Vector: x={dx}, y={dy}")
                # คำนวณจุดหมายจริงในโลกของเต่า (จุดปัจจุบัน + ระยะที่สั่ง)
                target_x = self.pose.x + dx
                target_y = self.pose.y + dy
                self.drive_to_target(target_x, target_y)

            # เมื่อครบทุกจุด หมุนกลับทิศเดิม
            self.get_logger().info("🔄 Aligning to Initial Heading...")
            self.align_to_angle(self.initial_theta)
            
            self.get_logger().info("🏁 Mission Complete!")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def drive_to_target(self, tx, ty):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            dx = tx - self.pose.x
            dy = ty - self.pose.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < self.stop_distance:
                break

            target_theta = math.atan2(dy, dx)
            err_th = self.normalize_angle(target_theta - self.pose.theta)

            cmd = Twist()
            cmd.linear.x = min(self.Kp_linear * dist, 1.0)
            cmd.angular.z = self.Kp_angular * err_th
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self.stop_robot()

    def align_to_angle(self, target_angle):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            err_th = self.normalize_angle(target_angle - self.pose.theta)
            if abs(err_th) < 0.02:
                break
            cmd = Twist()
            cmd.angular.z = self.Kp_angular * err_th
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self.stop_robot()

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()