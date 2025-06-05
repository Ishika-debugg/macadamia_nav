import rclpy
from rclpy.node import Node
from enum import Enum
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion

class NavigationState(Enum):
    INITIALIZING = 0
    FOLLOWING_ROW = 1
    TURNING_AT_END = 2
    MOVING_TO_NEXT_ROW = 3
    RETURNING_HOME = 4
    COMPLETED = 5

class MacadamiaNavigator(Node):
    def __init__(self):
        super().__init__('macadamia_navigator')

        self.current_state = NavigationState.INITIALIZING
        self.robot_pose = None
        self.start_pose = None

        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('row_distance', 1.2)
        self.declare_parameter('safe_distance', 0.6)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.row_following_distance = self.get_parameter('row_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value

        self.current_row = 0
        self.total_rows = 3
        self.row_spacing = 2.5
        self.row_length = 8.0

        self.laser_data = None
        self.obstacle_detected = False
        self.row_end_detected = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)

        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.state_start_time = self.get_clock().now()

        self.get_logger().info("Macadamia Navigator initialized")

    def laser_callback(self, msg):
        self.laser_data = msg
        self.analyze_laser_data()

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
            self.get_logger().info("Start position recorded")

    def analyze_laser_data(self):
        if self.laser_data is None:
            return
        ranges = np.array(self.laser_data.ranges)
        ranges = ranges[np.isfinite(ranges)]
        if len(ranges) == 0:
            return
        n_ranges = len(ranges)
        front = ranges[int(n_ranges * 0.4):int(n_ranges * 0.6)]
        self.obstacle_detected = len(front) > 0 and np.min(front) < self.safe_distance
        left = ranges[:n_ranges // 4]
        right = ranges[3 * n_ranges // 4:]
        left_clear = len(left) == 0 or np.min(left) > self.row_following_distance * 1.5
        right_clear = len(right) == 0 or np.min(right) > self.row_following_distance * 1.5
        self.row_end_detected = self.obstacle_detected and left_clear and right_clear

    def follow_row(self):
        if self.laser_data is None:
            return Twist()
        ranges = np.array(self.laser_data.ranges)
        n_ranges = len(ranges)
        right = ranges[int(n_ranges * 0.75):int(n_ranges * 0.95)]
        right_valid = right[np.isfinite(right)]
        cmd = Twist()
        if len(right_valid) > 3:
            current_dist = np.mean(right_valid)
            error = current_dist - self.row_following_distance
            cmd.angular.z = -0.8 * error
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
            reduction = 1.0 - 0.5 * abs(cmd.angular.z) / self.angular_speed
            cmd.linear.x = self.linear_speed * reduction
        else:
            cmd.linear.x = self.linear_speed * 0.5
        return cmd

    def return_home(self):
        if not self.start_pose or not self.robot_pose:
            return Twist()
        dx = self.start_pose.position.x - self.robot_pose.position.x
        dy = self.start_pose.position.y - self.robot_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        cmd = Twist()
        if dist < 0.3:
            self.get_logger().info("Reached home")
            self.current_state = NavigationState.COMPLETED
            return cmd
        yaw_target = math.atan2(dy, dx)
        q = self.robot_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        error = yaw_target - yaw
        error = (error + math.pi) % (2 * math.pi) - math.pi
        cmd.angular.z = 1.0 * error
        if abs(error) < 0.3:
            cmd.linear.x = min(self.linear_speed, 0.5 * dist)
        return cmd

    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now()
        if self.current_state == NavigationState.INITIALIZING:
            self.get_logger().info_once("Searching for tree row...")
            if self.find_tree_row():
                self.current_state = NavigationState.FOLLOWING_ROW
                self.state_start_time = now
                self.get_logger().info("Row found, following...")
            else:
                cmd.angular.z = 0.2
        elif self.current_state == NavigationState.FOLLOWING_ROW:
            if self.row_end_detected:
                self.get_logger().info("Row end reached")
                self.current_state = NavigationState.TURNING_AT_END
                self.state_start_time = now
            else:
                cmd = self.follow_row()
        elif self.current_state == NavigationState.TURNING_AT_END:
            cmd.angular.z = self.angular_speed
            if (now - self.state_start_time).nanoseconds / 1e9 > (3.14 / self.angular_speed):
                self.current_row += 1
                if self.current_row >= self.total_rows:
                    self.current_state = NavigationState.RETURNING_HOME
                else:
                    self.current_state = NavigationState.MOVING_TO_NEXT_ROW
                self.state_start_time = now
        elif self.current_state == NavigationState.MOVING_TO_NEXT_ROW:
            cmd.linear.x = self.linear_speed * 0.7
            move_time = self.row_spacing / (self.linear_speed * 0.7)
            if (now - self.state_start_time).nanoseconds / 1e9 > move_time:
                self.current_state = NavigationState.FOLLOWING_ROW
                self.state_start_time = now
        elif self.current_state == NavigationState.RETURNING_HOME:
            cmd = self.return_home()
        elif self.current_state == NavigationState.COMPLETED:
            cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        msg = String()
        msg.data = f"State: {self.current_state.name}, Row: {self.current_row + 1}/{self.total_rows}"
        self.status_pub.publish(msg)

    def find_tree_row(self):
        if self.laser_data is None:
            return False
        ranges = np.array(self.laser_data.ranges)
        n = len(ranges)
        right = ranges[3*n//4:]
        left = ranges[:n//4]
        right_valid = right[np.isfinite(right)]
        left_valid = left[np.isfinite(left)]
        if len(right_valid) > 5 and np.std(right_valid) < 0.5 and 0.8 < np.mean(right_valid) < 3.0:
            self.get_logger().info("Tree row detected on right")
            return True
        if len(left_valid) > 5 and np.std(left_valid) < 0.5 and 0.8 < np.mean(left_valid) < 3.0:
            self.get_logger().info("Tree row detected on left")
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = MacadamiaNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
