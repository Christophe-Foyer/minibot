#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import threading
from .DRV8825 import DRV8825


class WaveshareStepperNode(Node):
    def __init__(self):
        super().__init__('waveshare_stepper_node')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_base', 0.160)  # Distance between wheels in meters
        self.declare_parameter('wheel_diameter', 0.200)  # Wheel diameter in meters
        self.declare_parameter('steps_per_revolution', 200)  # Steps per motor revolution (full steps)
        self.declare_parameter('microstep_mode', 'fullstep')  # fullstep, halfstep, 1/4step, 1/8step, 1/16step, 1/32step
        self.declare_parameter('max_speed', 1.0)  # Max linear speed in m/s
        self.declare_parameter('acceleration', 1.0)  # Acceleration in m/s²
        self.declare_parameter('step_delay', 0.005)  # Delay between steps (seconds)
        self.declare_parameter('control_frequency', 20.0)  # Control loop frequency (Hz)
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.steps_per_revolution = self.get_parameter('steps_per_revolution').get_parameter_value().integer_value
        self.microstep_mode = self.get_parameter('microstep_mode').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value
        self.step_delay = self.get_parameter('step_delay').get_parameter_value().double_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # Microstep multipliers
        self.microstep_multipliers = {
            'fullstep': 1,
            'halfstep': 2,
            '1/4step': 4,
            '1/8step': 8,
            '1/16step': 16,
            '1/32step': 32
        }
        
        # Calculate actual steps per revolution based on microstepping
        self.actual_steps_per_rev = self.steps_per_revolution * self.microstep_multipliers.get(self.microstep_mode, 1)
        
        # Calculate wheel circumference and conversion factors
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.steps_per_meter = self.actual_steps_per_rev / self.wheel_circumference
        
        # Initialize DRV8825 motors
        self.motor_left = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
        self.motor_right = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))
        
        # Set microstepping mode
        self.motor_left.SetMicroStep('softward', self.microstep_mode)
        self.motor_right.SetMicroStep('softward', self.microstep_mode)
        
        # Motor state variables
        self.target_left_speed = 0.0   # Target speeds in m/s
        self.target_right_speed = 0.0
        self.current_left_speed = 0.0  # Current speeds in m/s
        self.current_right_speed = 0.0
        
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
        self.speed_timer = self.create_timer(1.0/self.control_frequency, self.update_speeds)
        
        self.get_logger().info(f"Waveshare Stepper Motor Node started")
        self.get_logger().info(f"Subscribing to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Wheel base: {self.wheel_base}m, Wheel diameter: {self.wheel_diameter}m")
        self.get_logger().info(f"Steps per revolution: {self.steps_per_revolution} (base), {self.actual_steps_per_rev} (with {self.microstep_mode})")
        self.get_logger().info(f"Steps per meter: {self.steps_per_meter:.2f}")
        self.get_logger().info(f"Control frequency: {self.control_frequency} Hz")

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
            max_delta = self.acceleration / self.control_frequency  # acceleration per control interval
            
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

    def motor_control_loop(self):
        """Main motor control loop running in separate thread"""
        last_time = time.time()
        left_step_accumulator = 0.0
        right_step_accumulator = 0.0
        
        while self.motor_thread_running and rclpy.ok():
            current_time = time.time()
            dt = current_time - last_time
            
            # Don't process if time step is too small
            if dt < 0.001:  # 1ms minimum
                time.sleep(0.001)
                continue
                
            last_time = current_time
            
            with self.speed_lock:
                left_speed = self.current_left_speed
                right_speed = self.current_right_speed
            
            # Calculate steps needed this iteration
            left_steps_needed = abs(left_speed) * self.steps_per_meter * dt
            right_steps_needed = abs(right_speed) * self.steps_per_meter * dt
            
            # Accumulate fractional steps
            left_step_accumulator += left_steps_needed
            right_step_accumulator += right_steps_needed
            
            # Execute whole steps
            left_steps = int(left_step_accumulator)
            right_steps = int(right_step_accumulator)
            
            left_step_accumulator -= left_steps
            right_step_accumulator -= right_steps
            
            # Step the motors if needed
            if left_steps > 0:
                direction = 'forward' if left_speed >= 0 else 'backward'
                self.step_motor_async(self.motor_left, direction, left_steps)
            
            if right_steps > 0:
                direction = 'forward' if right_speed >= 0 else 'backward'
                self.step_motor_async(self.motor_right, direction, right_steps)
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.01)  # 10ms

    def step_motor_async(self, motor, direction, steps):
        """Step motor without blocking - simplified version of TurnStep"""
        try:
            # Set direction and enable
            if direction == 'forward':
                motor.digital_write(motor.enable_pin, 1)
                motor.digital_write(motor.dir_pin, 0)
            elif direction == 'backward':
                motor.digital_write(motor.enable_pin, 1)
                motor.digital_write(motor.dir_pin, 1)
            else:
                motor.digital_write(motor.enable_pin, 0)
                return
            
            # Step the motor
            for i in range(steps):
                motor.digital_write(motor.step_pin, True)
                time.sleep(self.step_delay)
                motor.digital_write(motor.step_pin, False)
                time.sleep(self.step_delay)
                
        except Exception as e:
            self.get_logger().error(f"Error stepping motor: {str(e)}")

    def stop_motors(self):
        """Emergency stop - immediately stop all motors"""
        with self.speed_lock:
            self.target_left_speed = 0.0
            self.target_right_speed = 0.0
            self.current_left_speed = 0.0
            self.current_right_speed = 0.0
        
        try:
            self.motor_left.Stop()
            self.motor_right.Stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {str(e)}")

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info("Shutting down stepper motor node...")
        
        self.motor_thread_running = False
        self.stop_motors()
        
        if self.motor_thread.is_alive():
            self.motor_thread.join(timeout=1.0)
        
        self.get_logger().info("Motors stopped")
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
