#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time

class MotionMediator:
    """
    Motion Mediator class for smooth transitions between different controllers
    """
    def __init__(self, node: Node):
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'diff_drive_controller/cmd_vel_unstamped', 10)
        
        # Current and target velocities
        self.current_velocity = Twist()
        self.target_velocity = Twist()
        
        # Acceleration limits
        self.max_linear_accel = 0.5  # Maximum linear acceleration (m/s²)
        self.max_angular_accel = 1.0  # Maximum angular acceleration (rad/s²)
        
        # Active controller tracking
        self.active_controller = None
        self.transition_active = False
        self.transition_progress = 0.0
        self.transition_time = 0.5  # Default transition time (seconds)
        self.transition_start_time = 0.0
        
        # Create timer (20Hz = 0.05s)
        self.update_timer = self.node.create_timer(0.05, self.update_velocity)
        self.node.get_logger().info('Motion Mediator initialized')
    
    def set_target_velocity(self, linear_x, angular_z, controller_name):
        """
        Set target velocity from a controller
        """
        # Only accept commands from active controller or during transition
        if self.active_controller != controller_name and not self.transition_active:
            return
            
        self.target_velocity.linear.x = linear_x
        self.target_velocity.angular.z = angular_z
    
    def set_active_controller(self, controller_name, transition_time=0.5):
        """
        Change active controller with smooth transition
        """
        if self.active_controller == controller_name:
            return  # Already active, do nothing
            
        self.node.get_logger().info(f'Controller transition: {self.active_controller} -> {controller_name}')
        
        # Start transition
        self.previous_controller = self.active_controller
        self.active_controller = controller_name
        self.transition_active = True
        self.transition_progress = 0.0
        self.transition_time = transition_time
        self.transition_start_time = time.monotonic()
    
    def update_velocity(self):
        """
        Velocity update timer callback (implements smooth velocity changes)
        """
        current_time = time.monotonic()
        
        self.node.get_logger().info('update_velocity')
        
        # Handle controller transitions
        if self.transition_active:
            elapsed = current_time - self.transition_start_time
            self.transition_progress = min(elapsed / self.transition_time, 1.0)
            
            if self.transition_progress >= 1.0:
                self.transition_active = False
        
        # Apply acceleration limits for smooth velocity changes
        
        # Linear velocity smoothing
        linear_diff = self.target_velocity.linear.x - self.current_velocity.linear.x
        max_linear_step = self.max_linear_accel * 0.05  # 0.05 is the update period
        linear_step = np.clip(linear_diff, -max_linear_step, max_linear_step)
        self.current_velocity.linear.x += linear_step
        
        # Angular velocity smoothing
        angular_diff = self.target_velocity.angular.z - self.current_velocity.angular.z
        max_angular_step = self.max_angular_accel * 0.05  # 0.05 is the update period
        angular_step = np.clip(angular_diff, -max_angular_step, max_angular_step)
        self.current_velocity.angular.z += angular_step
        
        # Publish the smoothed cmd_vel
        self.cmd_vel_pub.publish(self.current_velocity)
    
    def stop(self):
        """
        Stop the robot smoothly
        """
        self.target_velocity.linear.x = 0.0
        self.target_velocity.linear.y = 0.0
        self.target_velocity.angular.z = 0.0
