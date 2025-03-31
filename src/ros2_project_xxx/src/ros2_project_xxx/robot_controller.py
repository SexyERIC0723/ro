
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
import math
import random
import numpy as np
import time
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.blue_box_sub = self.create_subscription(
            Float32MultiArray,
            'blue_box_detection',
            self.blue_box_callback,
            10)
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.current_pose = None
        self.blue_box_detected = False
        self.blue_box_position = None
        self.blue_box_size = None
        self.exploring = False
        self.navigating_to_blue = False
        
        self.timer = self.create_timer(1.0, self.exploration_timer_callback)
        
        self.get_logger().info('Robot controller initialized')
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def blue_box_callback(self, msg):
        data = msg.data
        
        if data[0] > 0.5:  # Box detected
            self.blue_box_detected = True
            self.blue_box_position = [data[1], data[2]]
            self.blue_box_size = data[3]
            
            if self.exploring and not self.navigating_to_blue:
                self.exploring = False
                self.navigate_to_blue_box(data[4])  # data[4] is distance estimate
        else:
            self.blue_box_detected = False
    
    def exploration_timer_callback(self):
        if not self.exploring and not self.navigating_to_blue:
            self.start_exploration()
    
    def start_exploration(self):
        self.exploring = True
        self.get_logger().info('Starting exploration')
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = random.uniform(-2.0, 2.0)
        goal_pose.pose.position.y = random.uniform(-2.0, 2.0)
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = random.uniform(-1.0, 1.0)
        goal_pose.pose.orientation.w = random.uniform(-1.0, 1.0)
        
        self.send_goal(goal_pose)
    
    def navigate_to_blue_box(self, distance_estimate):
        self.navigating_to_blue = True
        self.get_logger().info('Navigating to blue box')
        
        if self.current_pose is None:
            self.get_logger().warn('No current pose available')
            self.navigating_to_blue = False
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = max(0.0, distance_estimate - 1.0)  # Stay 1m away from the box
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.send_goal(goal_pose)
    
    def send_goal(self, goal_pose):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info('Sending goal')
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            if self.navigating_to_blue:
                self.navigating_to_blue = False
            if self.exploring:
                self.exploring = False
            return
        
        self.get_logger().info('Goal accepted')
        
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # Succeeded
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
        
        if self.navigating_to_blue:
            self.navigating_to_blue = False
            if status == 4:
                self.get_logger().info('Reached blue box, stopping exploration')
                self.timer.cancel()  # Stop the exploration timer
        
        if self.exploring:
            self.exploring = False
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        pass

def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
