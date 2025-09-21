#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import threading
from .DRV8825 import DRV8825


class MotorThread:
    """Individual motor control thread"""
    def __init__(self, motor, name, invert=False, steps_per_meter=1000, step_delay=0.005, timeout=2.0):
        self.motor = motor
        self.name = name
        self.invert = invert
        self.steps_per_meter = steps_per_meter
        self.step_delay = step_delay
        self.timeout = timeout
        
        # Thread control
        self.velocity = 0.0  # Current target velocity in m/s
        self.last_command_time = time.time()
        self.running = True
        self.velocity_lock = threading.Lock()
        
        # Start the thread
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()
    
    def set_velocity(self, velocity):
        """Set target velocity for this motor"""
        with self.velocity_lock:
            self.velocity = velocity
            self.last_command_time = time.time()
    
    def stop(self):
        """Stop the motor thread"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.motor.Stop()
    
    def _run(self):
        """Main motor control loop"""
        while self.running:
            current_time = time.time()
            
            with self.velocity_lock:
                vel = self.velocity
                last_cmd_time = self.last_command_time
            
            # Check for timeout
            if current_time - last_cmd_time > self.timeout:
                vel = 0.0  # Stop if command is stale
            
            if abs(vel) < 0.001:  # Motor is stopped
                self.motor.Stop()
                time.sleep(0.1)  # Check every 100ms when stopped
                continue
            
            # Calculate step timing
            abs_vel = abs(vel)
            steps_per_sec = abs_vel * self.steps_per_meter
            
            if steps_per_sec > 0:
                time_between_steps = 1.0 / steps_per_sec
                time_between_steps = max(time_between_steps, self.step_delay * 2)  # Minimum step time
            else:
                time.sleep(0.1)
                continue
            
            # Determine direction
            direction = 'forward' if vel >= 0 else 'backward'
            if self.invert:
                direction = 'backward' if direction == 'forward' else 'forward'
            
            # Set motor direction and enable
            self.motor.digital_write(self.motor.enable_pin, 1)
            if direction == 'forward':
                self.motor.digital_write(self.motor.dir_pin, 0)
            else:
                self.motor.digital_write(self.motor.dir_pin, 1)
            
            # Single step
            self.motor.digital_write(self.motor.step_pin, True)
            time.sleep(self.step_delay)
            self.motor.digital_write(self.motor.step_pin, False)
            
            # Wait for next step
            remaining_time = time_between_steps - self.step_delay
            if remaining_time > 0:
                time.sleep(remaining_time)


class WaveshareStepperNode(Node):
    def __init__(self):
        super().__init__('waveshare_stepper_node')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_base', 0.16)  # Distance between wheels in meters
        self.declare_parameter('wheel_diameter', 0.065)  # Wheel diameter in meters
        self.declare_parameter('steps_per_revolution', 200)  # Steps per motor revolution (full steps)
        self.declare_parameter('microstep_mode', 'fullstep')  # fullstep, halfstep, 1/4step, 1/8step, 1/16step, 1/32step
        self.declare_parameter('max_speed', 1.0)  # Max linear speed in m/s
        self.declare_parameter('step_delay', 0.001)  # Delay between steps (seconds)
        self.declare_parameter('motor_timeout', 2.0)  # Time in seconds before motor stops if no commands
        self.declare_parameter('invert_left_motor', False)  # Invert left motor direction
        self.declare_parameter('invert_right_motor', True)  # Invert right motor direction (default for differential drive)
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.steps_per_revolution = self.get_parameter('steps_per_revolution').get_parameter_value().integer_value
        self.microstep_mode = self.get_parameter('microstep_mode').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.step_delay = self.get_parameter('step_delay').get_parameter_value().double_value
        self.motor_timeout = self.get_parameter('motor_timeout').get_parameter_value().double_value
        self.invert_left_motor = self.get_parameter('invert_left_motor').get_parameter_value().bool_value
        self.invert_right_motor = self.get_parameter('invert_right_motor').get_parameter_value().bool_value
        
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
        motor_left_hw = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
        motor_right_hw = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))
        
        # Set microstepping mode
        motor_left_hw.SetMicroStep('softward', self.microstep_mode)
        motor_right_hw.SetMicroStep('softward', self.microstep_mode)
        
        # Create motor threads
        self.motor_left = MotorThread(
            motor=motor_left_hw,
            name="left",
            invert=self.invert_left_motor,
            steps_per_meter=self.steps_per_meter,
            step_delay=self.step_delay,
            timeout=self.motor_timeout
        )
        
        self.motor_right = MotorThread(
            motor=motor_right_hw,
            name="right", 
            invert=self.invert_right_motor,
            steps_per_meter=self.steps_per_meter,
            step_delay=self.step_delay,
            timeout=self.motor_timeout
        )
        
        # Subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info(f"Waveshare Stepper Motor Node started")
        self.get_logger().info(f"Subscribing to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Wheel base: {self.wheel_base}m, Wheel diameter: {self.wheel_diameter}m")
        self.get_logger().info(f"Steps per revolution: {self.steps_per_revolution} (base), {self.actual_steps_per_rev} (with {self.microstep_mode})")
        self.get_logger().info(f"Steps per meter: {self.steps_per_meter:.2f}")
        self.get_logger().info(f"Motor timeout: {self.motor_timeout}s")
        self.get_logger().info(f"Motor inversions - Left: {self.invert_left_motor}, Right: {self.invert_right_motor}")

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands - simple conversion and dispatch"""
        linear_vel = msg.linear.x  # Forward/backward velocity
        angular_vel = msg.angular.z  # Rotational velocity
        
        # Convert to differential drive wheel speeds
        left_speed = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_speed = linear_vel + (angular_vel * self.wheel_base / 2.0)
        
        # Limit speeds
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        
        # Send velocities to motor threads
        self.motor_left.set_velocity(left_speed)
        self.motor_right.set_velocity(right_speed)
        
        self.get_logger().debug(f"Cmd: linear={linear_vel:.2f}, angular={angular_vel:.2f} -> "
                               f"left={left_speed:.2f}, right={right_speed:.2f}")

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info("Shutting down stepper motor node...")
        
        # Stop motor threads
        self.motor_left.stop()
        self.motor_right.stop()
        
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
