#!/usr/bin/env python3

import cv2
import json
from flask import Flask, Response
from flask_socketio import SocketIO
from Waveshare_Stepper_Motor_HAT import StepperMotor
from dcam710 import DCAM710Camera

# ==== Setup ====
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Stepper setup
left_motor = StepperMotor(dir_pin=13, step_pin=19, enable_pin=12,
                          microstep_pins=(16,17,20), step_delay=0.002)
right_motor = StepperMotor(dir_pin=24, step_pin=18, enable_pin=4,
                           microstep_pins=(21,22,27), step_delay=0.002)

# Camera
dcam710 = DCAM710Camera(device_index=0)

# OpenCV video capture (RGB stream from Dcam710)
# TODO: Should I use the object above?
cap = cv2.VideoCapture(0)  # /dev/video0

def get_depth_frame():
    """
    Stub for Pico Zense Dcam710 depth grab.
    Replace with actual SDK call.
    Should return a 2D NumPy array (uint16).
    """
    depth = dcam710.get_depth_frame()
    return depth

# ==== HTTP Routes ====

@app.route("/rgb.jpg")
def rgb_image():
    ret, frame = cap.read()
    if not ret:
        return "Camera error", 500
    _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return Response(jpeg.tobytes(), mimetype="image/jpeg")

@app.route("/depth.jpg")
def depth_image():
    depth = get_depth_frame()
    # Normalize for visualization
    depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)  # scale 0–255
    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    _, jpeg = cv2.imencode(".jpg", depth_vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return Response(jpeg.tobytes(), mimetype="image/jpeg")

# ==== WebSocket Control ====

@socketio.on("cmd")
def handle_cmd(message):
    """
    Expects JSON like:
    { "linear": 0.2, "angular": 0.1 }
    """
    try:
        data = json.loads(message)
        linear = float(data.get("linear", 0))
        angular = float(data.get("angular", 0))

        # Simple differential drive model
        left_speed = linear - angular
        right_speed = linear + angular

        # Send to motors (convert to steps/s or PWM as needed)
        left_motor.run(left_speed)
        right_motor.run(right_speed)

        print(f"CMD: linear={linear}, angular={angular} -> L={left_speed}, R={right_speed}")

    except Exception as e:
        print("Bad cmd:", e)

# ==== Main ====

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
