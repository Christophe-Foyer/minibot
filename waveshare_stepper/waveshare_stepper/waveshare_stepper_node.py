#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import threading
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available. Running in simulation mode.")


class WaveshareStepperNode(Node):
    def __init__(self):
        super().__init__('waveshare_stepper_node')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_base', 0.16)  # Distance between wheels in meters
        self.declare_parameter('wheel_diameter', 0.065)  # Wheel diameter in meters
        self.declare_parameter('steps_per_revolution', 200)  # Steps per motor revolution
        self.declare_parameter('max_speed', 1.0)  # Max linear speed in m/s
        self.declare_parameter('acceleration', 2.0)  # Acceleration in m/s²
        self.declare_parameter('step_delay', 0.001)  # Minimum delay between steps (seconds)
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.steps_per_revolution = self.get_parameter('steps_per_revolution').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value
        self.step_delay = self.get_parameter('step_delay').get_parameter_value().double_value
        
        # GPIO Pin definitions for Waveshare Stepper Motor HAT
        # Motor A (Left wheel)
        self.MOTOR_A_DIR = 13
        self.MOTOR_A_STEP = 19
        self.MOTOR_A_EN = 12
        
        # Motor B (Right wheel)
        self.MOTOR_B_DIR = 24
        self.MOTOR_B_STEP = 18
        self.MOTOR_B_EN = 4
        
        # Calculate wheel circumference and conversion factors
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.steps_per_meter = self.steps_per_revolution / self.wheel_circumference
        
        # Initialize GPIO
        self.init_gpio()
        
        # Motor state variables
        self.target_left_speed = 0.0   # Target speeds in m/s
        self.target_right_speed = 0.0
        self.current_left_speed = 0.0  # Current speeds in m/s
        self.current_right_speed = 0.0
        self.left_direction = True     # True = forward, False = backward
        self.right_direction = True
        
        # Threading control
        self.motor_thread_running = True
        self.speed_lock = threading.Lock()
        
        # Subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        # Start motor control thread
        self.motor_thread = threading.Thread(target=self.motor_control_loop)
        self.motor_thread.daemon = True
        self.motor_thread.start()
        
        # Timer for speed ramping (smoother acceleration)
        self.speed_timer = self.create_timer(0.05, self.update_speeds)  # 20Hz
        
        self.get_logger().info(f"Waveshare Stepper Motor Node started")
        self.get_logger().info(f"Subscribing to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Wheel base: {self.wheel_base}m, Wheel diameter: {self.wheel_diameter}m")
        self.get_logger().info(f"Steps per revolution: {self.steps_per_revolution}")
        self.get_logger().info(f"GPIO available: {GPIO_AVAILABLE}")

    def init_gpio(self):
        """Initialize GPIO pins"""
        if not GPIO_AVAILABLE:
            self.get_logger().warn("GPIO not available - running in simulation mode")
            return
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup motor A pins
            GPIO.setup(self.MOTOR_A_DIR, GPIO.OUT)
            GPIO.setup(self.MOTOR_A_STEP, GPIO.OUT)
            GPIO.setup(self.MOTOR_A_EN, GPIO.OUT)
            
            # Setup motor B pins  
            GPIO.setup(self.MOTOR_B_DIR, GPIO.OUT)
            GPIO.setup(self.MOTOR_B_STEP, GPIO.OUT)
            GPIO.setup(self.MOTOR_B_EN, GPIO.OUT)
            
            # Enable motors (active low)
            GPIO.output(self.MOTOR_A_EN, GPIO.LOW)
            GPIO.output(self.MOTOR_B_EN, GPIO.LOW)
            
            # Set initial direction (forward)
            GPIO.output(self.MOTOR_A_DIR, GPIO.HIGH)
            GPIO.output(self.MOTOR_B_DIR, GPIO.HIGH)
            
            self.get_logger().info("GPIO initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {str(e)}")
            GPIO_AVAILABLE = False

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        linear_vel = msg.linear.x  # Forward/backward velocity
        angular_vel = msg.angular.z  # Rotational velocity
        
        # Convert to differential drive wheel speeds
        # v_left = linear - (angular * wheelbase / 2)
        # v_right = linear + (angular * wheelbase / 2)
        left_speed = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_speed = linear_vel + (angular_vel * self.wheel_base / 2.0)
        
        # Limit speeds
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        
        with self.speed_lock:
            self.target_left_speed = left_speed
            self.target_right_speed = right_speed
        
        self.get_logger().debug(f"Cmd: linear={linear_vel:.2f}, angular={angular_vel:.2f} -> "
                               f"left={left_speed:.2f}, right={right_speed:.2f}")

    def update_speeds(self):
        """Gradually update current speeds toward target speeds (acceleration limiting)"""
        with self.speed_lock:
            max_delta = self.acceleration * 0.05  # 20Hz timer = 0.05s interval
            
            # Update left speed
            speed_diff = self.target_left_speed - self.current_left_speed
            if abs(speed_diff) <= max_delta:
                self.current_left_speed = self.target_left_speed
            else:
                self.current_left_speed += max_delta if speed_diff > 0 else -max_delta
            
            # Update right speed
            speed_diff = self.target_right_speed - self.current_right_speed
            if abs(speed_diff) <= max_delta:
                self.current_right_speed = self.target_right_speed
            else:
                self.current_right_speed += max_delta if speed_diff > 0 else -max_delta
            
            # Update directions
            self.left_direction = self.current_left_speed >= 0
            self.right_direction = self.current_right_speed >= 0

    def motor_control_loop(self):
        """Main motor control loop running in separate thread"""
        last_time = time.time()
        left_step_accumulator = 0.0
        right_step_accumulator = 0.0
        
        while self.motor_thread_running and rclpy.ok():
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            with self.speed_lock:
                left_speed = abs(self.current_left_speed)
                right_speed = abs(self.current_right_speed)
                left_dir = self.left_direction
                right_dir = self.right_direction
            
            if GPIO_AVAILABLE:
                # Set motor directions
                GPIO.output(self.MOTOR_A_DIR, GPIO.HIGH if left_dir else GPIO.LOW)
                GPIO.output(self.MOTOR_B_DIR, GPIO.HIGH if right_dir else GPIO.LOW)
            
            # Calculate steps needed this iteration
            left_steps_needed = left_speed * self.steps_per_meter * dt
            right_steps_needed = right_speed * self.steps_per_meter * dt
            
            # Accumulate fractional steps
            left_step_accumulator += left_steps_needed
            right_step_accumulator += right_steps_needed
            
            # Execute whole steps
            left_steps = int(left_step_accumulator)
            right_steps = int(right_step_accumulator)
            
            left_step_accumulator -= left_steps
            right_step_accumulator -= right_steps
            
            # Step the motors
            max_steps = max(left_steps, right_steps)
            if max_steps > 0:
                self.step_motors_synchronized(left_steps, right_steps, max_steps)
            
            # Small delay to prevent excessive CPU usage
            time.sleep(self.step_delay)

    def step_motors_synchronized(self, left_steps, right_steps, max_steps):
        """Step both motors in a synchronized manner"""
        if not GPIO_AVAILABLE:
            return
            
        left_step_interval = max_steps / left_steps if left_steps > 0 else float('inf')
        right_step_interval = max_steps / right_steps if right_steps > 0 else float('inf')
        
        left_next_step = left_step_interval if left_steps > 0 else float('inf')
        right_next_step = right_step_interval if right_steps > 0 else float('inf')
        
        try:
            for step in range(max_steps):
                # Step left motor if needed
                if step >= left_next_step:
                    GPIO.output(self.MOTOR_A_STEP, GPIO.HIGH)
                    time.sleep(self.step_delay / 2)
                    GPIO.output(self.MOTOR_A_STEP, GPIO.LOW)
                    left_next_step += left_step_interval
                
                # Step right motor if needed
                if step >= right_next_step:
                    GPIO.output(self.MOTOR_B_STEP, GPIO.HIGH)
                    time.sleep(self.step_delay / 2)
                    GPIO.output(self.MOTOR_B_STEP, GPIO.LOW)
                    right_next_step += right_step_interval
                
                time.sleep(self.step_delay / 2)
                
        except Exception as e:
            self.get_logger().error(f"Error stepping motors: {str(e)}")

    def stop_motors(self):
        """Emergency stop - immediately stop all motors"""
        with self.speed_lock:
            self.target_left_speed = 0.0
            self.target_right_speed = 0.0
            self.current_left_speed = 0.0
            self.current_right_speed = 0.0
        
        if GPIO_AVAILABLE:
            try:
                GPIO.output(self.MOTOR_A_EN, GPIO.HIGH)  # Disable motors
                GPIO.output(self.MOTOR_B_EN, GPIO.HIGH)
            except Exception:
                pass

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info("Shutting down stepper motor node...")
        
        self.motor_thread_running = False
        self.stop_motors()
        
        if self.motor_thread.is_alive():
            self.motor_thread.join(timeout=1.0)
        
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
                self.get_logger().info("GPIO cleaned up")
            except Exception as e:
                self.get_logger().error(f"Error cleaning up GPIO: {str(e)}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = WaveshareStepperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
