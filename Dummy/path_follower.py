import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import json
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        self.Kp_linear = 1.5
        self.Kp_angular = 6.0
        self.stop_distance = 0.1
        
        self.pose = None
        self.path_queue = []
        self.current_target = None
        self.is_moving = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.path_sub = self.create_subscription(String, '/navigation/path', self.path_callback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ Path Follower Ready! (With STOP system)")

    def pose_callback(self, msg):
        self.pose = msg

    def path_callback(self, msg):
        try:
            new_path = json.loads(msg.data)
            
            # ✅✅✅ เพิ่ม Logic STOP ตรงนี้
            if len(new_path) == 0:
                self.get_logger().warn("🛑 STOP COMMAND RECEIVED!")
                self.path_queue = []       # ล้างคิว
                self.current_target = None # ยกเลิกเป้าหมาย
                self.is_moving = False     # หยุดสถานะเดิน
                self.stop_robot()          # สั่งล้อหยุดหมุน
                return

            if new_path:
                self.get_logger().info(f"📥 Received Path: {len(new_path)} points")
                self.path_queue = new_path
                self.current_target = self.path_queue.pop(0)
                self.is_moving = True
        except Exception as e:
            self.get_logger().error(f"Path Error: {e}")

    def control_loop(self):
        if not self.pose or not self.is_moving or not self.current_target:
            return

        tx, ty = self.current_target
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        target_theta = math.atan2(dy, dx)
        angle_err = target_theta - self.pose.theta

        while angle_err > math.pi: angle_err -= 2 * math.pi
        while angle_err < -math.pi: angle_err += 2 * math.pi

        if distance < self.stop_distance:
            self.get_logger().info(f"📍 Reached ({tx}, {ty})")
            if self.path_queue:
                self.current_target = self.path_queue.pop(0)
            else:
                self.get_logger().info("🏁 Path Completed!")
                self.is_moving = False
                self.stop_robot()
            return

        cmd = Twist()
        cmd.linear.x = min(self.Kp_linear * distance, 2.0)
        cmd.angular.z = self.Kp_angular * angle_err
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        # สั่งความเร็ว 0 ทันที
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()