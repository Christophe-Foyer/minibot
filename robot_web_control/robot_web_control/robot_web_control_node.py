#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import threading
import time
import base64
import json
from flask import Flask, render_template_string, jsonify, request
from flask_socketio import SocketIO, emit
import socketio


class RobotControlWebNode(Node):
    def __init__(self):
        super().__init__('robot_control_web_node')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('flip_image', False)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.web_port = self.get_parameter('web_port').get_parameter_value().integer_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.flip_image = self.get_parameter('flip_image').get_parameter_value().bool_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Image subscriber
        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Store latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()
            
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Control variables
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        # Initialize Flask app and SocketIO
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'ros2_robot_secret'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        self.setup_web_routes()
        
        # Start web server thread
        self.web_thread = threading.Thread(target=self.run_web_server)
        self.web_thread.daemon = True
        self.web_thread.start()
        
        # Timer for publishing control commands
        self.control_timer = self.create_timer(0.1, self.publish_cmd_vel)
        
        self.get_logger().info(f"Robot Control Web Node started on port {self.web_port}")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing cmd_vel to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Web interface: http://localhost:{self.web_port}")

    def setup_web_routes(self):
        """Setup Flask routes and SocketIO event handlers"""
        
        @self.app.route('/')
        def index():
            return render_template_string(WEB_UI_TEMPLATE, 
                                        max_linear=self.max_linear_speed,
                                        max_angular=self.max_angular_speed)
        
        @self.socketio.on('connect')
        def handle_connect():
            self.get_logger().info('Web client connected')
            
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.get_logger().info('Web client disconnected')
            # Stop robot when client disconnects
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            
        @self.socketio.on('robot_control')
        def handle_robot_control(data):
            self.linear_speed = float(data.get('linear', 0.0))
            self.angular_speed = float(data.get('angular', 0.0))
            
        @self.socketio.on('stop_robot')
        def handle_stop():
            self.linear_speed = 0.0
            self.angular_speed = 0.0

    def image_callback(self, msg):
        """Callback for image subscription"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Optionally flip vertically
            if self.flip_image:
                cv_image = cv2.flip(cv_image, 0)  # 0 = vertical flip
            
            # Store the frame with thread safety
            with self.frame_lock:
                self.latest_frame = cv_image
            
            # Encode frame as JPEG for web streaming
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Emit frame to web clients
            self.socketio.emit('video_frame', {'frame': frame_base64})
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_cmd_vel(self):
        """Publish velocity commands to ROS2 topic"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(twist)

    def run_web_server(self):
        """Run Flask-SocketIO web server"""
        self.socketio.run(self.app, host='0.0.0.0', port=self.web_port, debug=False, allow_unsafe_werkzeug=True)

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        super().destroy_node()


# HTML template for the web UI
WEB_UI_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>ROS2 Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .video-container {
            text-align: center;
            margin-bottom: 20px;
        }
        #video-stream {
            max-width: 100%;
            border: 2px solid #ddd;
            border-radius: 5px;
        }
        .controls {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .control-panel {
            padding: 15px;
            border: 1px solid #ddd;
            border-radius: 5px;
            background: #f9f9f9;
        }
        .control-buttons {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 10px;
            margin-bottom: 15px;
        }
        .control-buttons button {
            padding: 15px;
            border: none;
            border-radius: 5px;
            background: #007bff;
            color: white;
            cursor: pointer;
            font-size: 16px;
        }
        .control-buttons button:hover {
            background: #0056b3;
        }
        .control-buttons button:active {
            background: #004085;
        }
        .empty { background: transparent !important; }
        .stop-btn {
            grid-column: span 3;
            background: #dc3545 !important;
        }
        .stop-btn:hover {
            background: #c82333 !important;
        }
        .slider-control {
            margin: 10px 0;
        }
        .slider-control label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }
        .slider-control input {
            width: 100%;
            margin-bottom: 5px;
        }
        .status {
            text-align: center;
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 20px;
        }
        .connected { background: #d4edda; color: #155724; }
        .disconnected { background: #f8d7da; color: #721c24; }
        @media (max-width: 768px) {
            .controls {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ROS2 Robot Control Interface</h1>
        
        <div id="status" class="status disconnected">Disconnected</div>
        
        <div class="video-container">
            <img id="video-stream" src="data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNjQwIiBoZWlnaHQ9IjQ4MCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48cmVjdCB3aWR0aD0iMTAwJSIgaGVpZ2h0PSIxMDAlIiBmaWxsPSIjZGRkIi8+PHRleHQgeD0iNTAlIiB5PSI1MCUiIGZvbnQtZmFtaWx5PSJBcmlhbCwgc2Fucy1zZXJpZiIgZm9udC1zaXplPSIxOCIgZmlsbD0iIzk5OSIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZHk9Ii4zZW0iPldhaXRpbmcgZm9yIHZpZGVvLi4uPC90ZXh0Pjwvc3ZnPg==" alt="Video Stream">
        </div>
        
        <div class="controls">
            <div class="control-panel">
                <h3>Manual Control</h3>
                <div class="control-buttons">
                    <div class="empty"></div>
                    <button onmousedown="sendControl({{ max_linear }}, 0)" onmouseup="stopRobot()" ontouchstart="sendControl({{ max_linear }}, 0)" ontouchend="stopRobot()">↑<br>Forward</button>
                    <div class="empty"></div>
                    
                    <button onmousedown="sendControl(0, {{ max_angular }})" onmouseup="stopRobot()" ontouchstart="sendControl(0, {{ max_angular }})" ontouchend="stopRobot()">↶<br>Left</button>
                    <button class="stop-btn" onclick="stopRobot()">⏹<br>STOP</button>
                    <button onmousedown="sendControl(0, -{{ max_angular }})" onmouseup="stopRobot()" ontouchstart="sendControl(0, -{{ max_angular }})" ontouchend="stopRobot()">↷<br>Right</button>
                    
                    <div class="empty"></div>
                    <button onmousedown="sendControl(-{{ max_linear }}, 0)" onmouseup="stopRobot()" ontouchstart="sendControl(-{{ max_linear }}, 0)" ontouchend="stopRobot()">↓<br>Backward</button>
                    <div class="empty"></div>
                </div>
            </div>
            
            <div class="control-panel">
                <h3>Precise Control</h3>
                <div class="slider-control">
                    <label for="linear-speed">Linear Speed: <span id="linear-value">0.0</span> m/s</label>
                    <input type="range" id="linear-speed" min="-{{ max_linear }}" max="{{ max_linear }}" step="0.1" value="0" oninput="updateControl()">
                </div>
                <div class="slider-control">
                    <label for="angular-speed">Angular Speed: <span id="angular-value">0.0</span> rad/s</label>
                    <input type="range" id="angular-speed" min="-{{ max_angular }}" max="{{ max_angular }}" step="0.1" value="0" oninput="updateControl()">
                </div>
                <button onclick="resetSliders()" style="width: 100%; padding: 10px; background: #6c757d; color: white; border: none; border-radius: 5px; cursor: pointer;">Reset</button>
            </div>
        </div>
    </div>

    <script>
        const socket = io();
        const statusDiv = document.getElementById('status');
        const videoStream = document.getElementById('video-stream');
        const linearSlider = document.getElementById('linear-speed');
        const angularSlider = document.getElementById('angular-speed');
        const linearValue = document.getElementById('linear-value');
        const angularValue = document.getElementById('angular-value');

        socket.on('connect', function() {
            statusDiv.textContent = 'Connected';
            statusDiv.className = 'status connected';
        });

        socket.on('disconnect', function() {
            statusDiv.textContent = 'Disconnected';
            statusDiv.className = 'status disconnected';
        });

        socket.on('video_frame', function(data) {
            videoStream.src = 'data:image/jpeg;base64,' + data.frame;
        });

        function sendControl(linear, angular) {
            socket.emit('robot_control', {linear: linear, angular: angular});
        }

        function stopRobot() {
            socket.emit('stop_robot');
            resetSliders();
        }

        function updateControl() {
            const linear = parseFloat(linearSlider.value);
            const angular = parseFloat(angularSlider.value);
            linearValue.textContent = linear.toFixed(1);
            angularValue.textContent = angular.toFixed(1);
            sendControl(linear, angular);
        }

        function resetSliders() {
            linearSlider.value = 0;
            angularSlider.value = 0;
            linearValue.textContent = '0.0';
            angularValue.textContent = '0.0';
            sendControl(0, 0);
        }

        // Prevent scrolling when using touch controls
        document.addEventListener('touchstart', function(e) {
            if (e.target.tagName === 'BUTTON') {
                e.preventDefault();
            }
        }, {passive: false});

        document.addEventListener('touchend', function(e) {
            if (e.target.tagName === 'BUTTON') {
                e.preventDefault();
            }
        }, {passive: false});
    </script>
</body>
</html>
'''


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotControlWebNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
