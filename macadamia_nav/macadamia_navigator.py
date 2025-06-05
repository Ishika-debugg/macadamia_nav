#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from enum import Enum

class NavigationState(Enum):
    INITIALIZING = 0
    SEARCHING_FOR_ROW = 1
    FOLLOWING_ROW = 2
    TURNING_AT_END = 3
    MOVING_TO_NEXT_ROW = 4
    RETURNING_HOME = 5
    COMPLETED = 6

class MacadamiaNavigator:
    def __init__(self):
        rospy.init_node('macadamia_navigator', anonymous=True)
        
        # Navigation state
        self.current_state = NavigationState.INITIALIZING
        self.robot_pose = None
        self.start_pose = None
        
        # Navigation parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.25)
        self.angular_speed = rospy.get_param('~angular_speed', 0.3)
        self.row_following_distance = rospy.get_param('~row_distance', 1.2)
        self.safe_distance = rospy.get_param('~safe_distance', 0.6)
        
        # Row navigation variables
        self.current_row = 0
        self.total_rows = 3  # Configurable
        self.row_spacing = 2.5  # meters between rows
        self.row_length = 8.0   # meters per row
        self.rows_completed = []
        
        # Control variables
        self.laser_data = None
        self.obstacle_detected = False
        self.row_end_detected = False
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/navigation/status', String, queue_size=1)
        
        # Subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        
        # Timers for state management
        self.last_turn_time = rospy.Time.now()
        self.state_start_time = rospy.Time.now()
        
        rospy.loginfo("Macadamia Navigator initialized")
        rospy.loginfo(f"Parameters: speed={self.linear_speed}, row_distance={self.row_following_distance}")
        
    def laser_callback(self, data):
        """Process laser scan data"""
        self.laser_data = data
        self.analyze_laser_data()
        
    def odometry_callback(self, data):
        """Update robot pose from odometry"""
        self.robot_pose = data.pose.pose
        if self.start_pose is None:
            self.start_pose = data.pose.pose
            rospy.loginfo("Start position recorded")
            
    def analyze_laser_data(self):
        """Analyze laser data for navigation decisions"""
        if self.laser_data is None:
            return
            
        ranges = np.array(self.laser_data.ranges)
        ranges = ranges[np.isfinite(ranges)]  # Remove inf values
        
        if len(ranges) == 0:
            return
            
        n_ranges = len(ranges)
        
        # Divide laser scan into sectors
        front_start = int(n_ranges * 0.4)
        front_end = int(n_ranges * 0.6)
        
        # Front sector (for obstacle detection)
        front_ranges = ranges[front_start:front_end]
        self.obstacle_detected = len(front_ranges) > 0 and np.min(front_ranges) < self.safe_distance
        
        # Check for row end (no obstacles on sides, obstacle ahead)
        left_ranges = ranges[:n_ranges//4]
        right_ranges = ranges[3*n_ranges//4:]
        
        left_clear = len(left_ranges) == 0 or np.min(left_ranges) > self.row_following_distance * 1.5
        right_clear = len(right_ranges) == 0 or np.min(right_ranges) > self.row_following_distance * 1.5
        
        self.row_end_detected = self.obstacle_detected and left_clear and right_clear
        
    def find_tree_row(self):
        """Detect and align with tree row using laser data"""
        if self.laser_data is None:
            return False
            
        ranges = np.array(self.laser_data.ranges)
        n_ranges = len(ranges)
        
        # Look for consistent obstacles on one side (tree line)
        right_ranges = ranges[3*n_ranges//4:]  # Right side
        left_ranges = ranges[:n_ranges//4]     # Left side
        
        # Filter out infinite values
        right_valid = right_ranges[np.isfinite(right_ranges)]
        left_valid = left_ranges[np.isfinite(left_ranges)]
        
        if len(right_valid) > 5:
            right_avg = np.mean(right_valid)
            right_std = np.std(right_valid)
            
            # Consistent distance indicates tree line
            if right_std < 0.5 and 0.8 < right_avg < 3.0:
                rospy.loginfo(f"Tree row detected on right side at {right_avg:.2f}m")
                return True
                
        if len(left_valid) > 5:
            left_avg = np.mean(left_valid)
            left_std = np.std(left_valid)
            
            if left_std < 0.5 and 0.8 < left_avg < 3.0:
                rospy.loginfo(f"Tree row detected on left side at {left_avg:.2f}m")
                return True
                
        return False
        
    def follow_row(self):
        """Main row following logic using wall-following algorithm"""
        if self.laser_data is None:
            return Twist()
            
        ranges = np.array(self.laser_data.ranges)
        n_ranges = len(ranges)
        
        # Get right side distances (assuming trees are on the right)
        right_start = int(n_ranges * 0.75)
        right_end = int(n_ranges * 0.95)
        right_ranges = ranges[right_start:right_end]
        
        # Filter valid readings
        right_valid = right_ranges[np.isfinite(right_ranges)]
        
        cmd = Twist()
        
        if len(right_valid) > 3:
            # Calculate average distance to tree line
            current_distance = np.mean(right_valid)
            
            # PID-like control to maintain desired distance
            distance_error = current_distance - self.row_following_distance
            
            # Proportional control gains
            kp_angular = 0.8
            kp_linear = 0.3
            
            # Angular velocity to maintain distance from trees
            cmd.angular.z = -kp_angular * distance_error
            
            # Limit angular velocity
            max_angular = self.angular_speed
            cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))
            
            # Linear velocity (reduce when turning)
            angular_factor = abs(cmd.angular.z) / max_angular
            speed_reduction = 1.0 - 0.5 * angular_factor
            cmd.linear.x = self.linear_speed * speed_reduction
            
        else:
            # No tree line detected, move forward slowly
            cmd.linear.x = self.linear_speed * 0.5
            
        return cmd
        
    def turn_around(self):
        """Execute 180-degree turn at end of row"""
        cmd = Twist()
        
        # Turn in place
        cmd.angular.z = self.angular_speed
        cmd.linear.x = 0.0
        
        return cmd
        
    def move_to_next_row(self):
        """Navigate to the next row"""
        cmd = Twist()
        
        # Simple forward movement to next row
        # In a real implementation, you might use more sophisticated path planning
        cmd.linear.x = self.linear_speed * 0.7
        cmd.angular.z = 0.0
        
        return cmd
        
    def return_home(self):
        """Navigate back to starting position"""
        if self.start_pose is None or self.robot_pose is None:
            return Twist()
            
        # Calculate vector to start position
        dx = self.start_pose.position.x - self.robot_pose.position.x
        dy = self.start_pose.position.y - self.robot_pose.position.y
        distance_to_start = math.sqrt(dx*dx + dy*dy)
        
        cmd = Twist()
        
        if distance_to_start < 0.3:  # Close enough to start
            rospy.loginfo("Reached starting position")
            self.current_state = NavigationState.COMPLETED
            return cmd
            
        # Calculate desired heading to start
        target_yaw = math.atan2(dy, dx)
        
        # Get current yaw
        current_quat = self.robot_pose.orientation
        _, _, current_yaw = euler_from_quaternion([
            current_quat.x, current_quat.y, current_quat.z, current_quat.w
        ])
        
        # Calculate yaw error
        yaw_error = target_yaw - current_yaw
        
        # Normalize angle to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
            
        # Control gains
        kp_yaw = 1.0
        kp_distance = 0.5
        
        # Angular control to face start position
        cmd.angular.z = kp_yaw * yaw_error
        
        # Linear control to move toward start
        if abs(yaw_error) < 0.3:  # Only move forward when roughly facing target
            cmd.linear.x = min(self.linear_speed, kp_distance * distance_to_start)
        
        return cmd
        
    def state_machine(self):
        """Main state machine for navigation"""
        cmd = Twist()
        current_time = rospy.Time.now()
        
        if self.current_state == NavigationState.INITIALIZING:
            rospy.loginfo("Initializing - looking for tree rows")
            if self.find_tree_row():
                self.current_state = NavigationState.FOLLOWING_ROW
                self.state_start_time = current_time
                rospy.loginfo("Tree row found - starting row following")
            else:
                # Slowly rotate to search for trees
                cmd.angular.z = 0.2
                
        elif self.current_state == NavigationState.FOLLOWING_ROW:
            if self.row_end_detected:
                rospy.loginfo("Row end detected - turning around")
                self.current_state = NavigationState.TURNING_AT_END
                self.state_start_time = current_time
                self.last_turn_time = current_time
            else:
                cmd = self.follow_row()
                
        elif self.current_state == NavigationState.TURNING_AT_END:
            cmd = self.turn_around()
            
            # Turn for approximately 180 degrees
            turn_duration = 3.14 / self.angular_speed  # Time for 180 degree turn
            if (current_time - self.state_start_time).to_sec() > turn_duration:
                self.current_row += 1
                
                if self.current_row >= self.total_rows:
                    rospy.loginfo("All rows completed - returning home")
                    self.current_state = NavigationState.RETURNING_HOME
                else:
                    rospy.loginfo(f"Moving to row {self.current_row + 1}")
                    self.current_state = NavigationState.MOVING_TO_NEXT_ROW
                    
                self.state_start_time = current_time
                
        elif self.current_state == NavigationState.MOVING_TO_NEXT_ROW:
            cmd = self.move_to_next_row()
            
            # Move for a set duration to reach next row
            move_duration = self.row_spacing / (self.linear_speed * 0.7)
            if (current_time - self.state_start_time).to_sec() > move_duration:
                rospy.loginfo("Reached next row - resuming row following")
                self.current_state = NavigationState.FOLLOWING_ROW
                self.state_start_time = current_time
                
        elif self.current_state == NavigationState.RETURNING_HOME:
            cmd = self.return_home()
            
        elif self.current_state == NavigationState.COMPLETED:
            rospy.loginfo("Navigation completed successfully!")
            cmd = Twist()  # Stop
            
        return cmd
        
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Starting navigation system...")
        
        while not rospy.is_shutdown():
            # Get navigation command from state machine
            cmd = self.state_machine()
            
            # Safety check - stop if no laser data for obstacle avoidance
            if self.laser_data is None:
                rospy.logwarn("No laser data - stopping for safety")
                cmd = Twist()
                
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
            # Publish status
            status_msg = String()
            status_msg.data = f"State: {self.current_state.name}, Row: {self.current_row + 1}/{self.total_rows}"
            self.status_pub.publish(status_msg)
            
            # Emergency stop if commanded
            if rospy.is_shutdown():
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                break
                
            rate.sleep()
            
        # Final stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("Navigation system stopped")

if __name__ == '__main__':
    try:
        navigator = MacadamiaNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted by user")
    except Exception as e:
        rospy.logerr(f"Navigation error: {e}")
    finally:
        # Ensure robot stops
        try:
            stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            rospy.sleep(0.1)  # Give time for publisher to connect
            stop_pub.publish(Twist())
        except:
            pass