import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import numpy as np

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator_node')
        
        # --- Initial State (เริ่มที่จุด 0,0) ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        
        # --- Config ---
        self.dt = 0.05 # 20 Hz
        self.noise_v = 0.0 # ใส่ Noise ได้ถ้าอยากให้เหมือนจริง (เช่น 0.01)
        self.noise_w = 0.0

        # --- ROS Setup ---
        self.pose_pub = self.create_publisher(Pose, '/turtle1/pose', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(self.dt, self.update_physics)
        
        self.get_logger().info("🐢 Robot Simulator Started! (MATLAB Physics Model)")

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_physics(self):
        # --- Physics Update (เหมือน MATLAB) ---
        # x_new = x_old + v * cos(theta) * dt
        
        # ใส่ Noise จำลองความคลาดเคลื่อน
        v_act = self.v + np.random.normal(0, self.noise_v) if self.noise_v > 0 else self.v
        w_act = self.w + np.random.normal(0, self.noise_w) if self.noise_w > 0 else self.w

        self.x += v_act * math.cos(self.theta) * self.dt
        self.y += v_act * math.sin(self.dt) * self.dt  # แก้ไข typo: sin(self.theta)
        self.y += v_act * math.sin(self.theta) * self.dt
        self.theta += w_act * self.dt

        # Normalize theta
        while self.theta > math.pi: self.theta -= 2 * math.pi
        while self.theta < -math.pi: self.theta += 2 * math.pi

        # --- Publish Pose ---
        pose = Pose()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta
        pose.linear_velocity = self.v  # ส่งค่านี้กลับไปโชว์ที่หน้าเว็บ
        pose.angular_velocity = self.w
        
        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()