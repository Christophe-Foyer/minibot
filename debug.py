import time
import numpy as np
from ctypes import cdll, c_int, POINTER, byref, c_char_p, c_void_p, Structure, c_uint16, c_uint8

cam = cdll.LoadLibrary("./VZense_python_wrapper/Lib/libvzense_api.so")

# --- Structs ---
class PsFrameReady(Structure):
    _fields_ = [
        ("depth", c_int),
        ("ir", c_int),
        ("rgb", c_int),
    ]

class PsFrame(Structure):
    _fields_ = [
        ("frameIndex", c_int),
        ("width", c_int),
        ("height", c_int),
        ("status", c_int),
        ("pixelFormat", c_int),
        ("dataLen", c_int),
        ("pFrameData", c_void_p),
    ]

# --- Function prototypes ---
cam.Ps2_Initialize.restype = c_int

cam.Ps2_GetDeviceCount.argtypes = [POINTER(c_int)]
cam.Ps2_GetDeviceCount.restype = c_int

cam.Ps2_OpenDevice.argtypes = [c_char_p, POINTER(c_void_p)]
cam.Ps2_OpenDevice.restype = c_int

cam.Ps2_CloseDevice.argtypes = [c_void_p]
cam.Ps2_CloseDevice.restype = c_int

cam.Ps2_StartStream.argtypes = [c_void_p]
cam.Ps2_StartStream.restype = c_int

cam.Ps2_StopStream.argtypes = [c_void_p]
cam.Ps2_StopStream.restype = c_int

cam.Ps2_SetWaitTimeOfReadNextFrame.argtypes = [c_void_p, c_int]
cam.Ps2_SetWaitTimeOfReadNextFrame.restype = c_int

cam.Ps2_ReadNextFrame.argtypes = [c_void_p, POINTER(PsFrameReady)]
cam.Ps2_ReadNextFrame.restype = c_int

cam.Ps2_GetFrame.argtypes = [c_void_p, c_int, POINTER(PsFrame)]
cam.Ps2_GetFrame.restype = c_int

ret = cam.Ps2_Initialize()
print("Init:", ret)

count = c_int()
ret = cam.Ps2_GetDeviceCount(byref(count))
print("ret:", ret, "count:", count.value)

handle = c_void_p()
ret = cam.Ps2_OpenDevice(b"0", byref(handle))  # "0" selects first device
print("OpenDevice ret:", ret, "handle:", handle)

ret = cam.Ps2_StartStream(handle)
print("StartStream:", ret)

# Set read timeout
cam.Ps2_SetWaitTimeOfReadNextFrame(handle, 2000)

# Allow camera pipeline to stabilize
time.sleep(3.0)

# --- Read frames ---
frame_ready = PsFrameReady()
while True:
    ret = cam.Ps2_ReadNextFrame(handle, byref(frame_ready))
    if ret == 0:
        if frame_ready.rgb or frame_ready.depth:
            print("Frame ready! Depth:", frame_ready.depth, "RGB:", frame_ready.rgb)
            break
    elif ret == -11:
        # Frame not ready yet
        time.sleep(0.01)
        continue
    else:
        print("ReadNextFrame error:", ret)
        break

# --- Get depth frame ---
if frame_ready.depth:
    depth_frame = PsFrame()
    ret = cam.Ps2_GetFrame(handle, 0, byref(depth_frame))  # 0 = PsDepthFrame
    if ret == 0 and depth_frame.pFrameData:
        buf_type = c_uint16 * (depth_frame.width * depth_frame.height)
        buf = buf_type.from_address(depth_frame.pFrameData)
        depth_np = np.ctypeslib.as_array(buf).reshape(depth_frame.height, depth_frame.width)
        print("Depth frame shape:", depth_np.shape, "min/max:", depth_np.min(), depth_np.max())

# --- Get RGB frame ---
if frame_ready.rgb:
    rgb_frame = PsFrame()
    ret = cam.Ps2_GetFrame(handle, 1, byref(rgb_frame))  # 1 = PsRGBFrame
    if ret == 0 and rgb_frame.pFrameData:
        buf_type = c_uint8 * (rgb_frame.width * rgb_frame.height * 3)
        buf = buf_type.from_address(rgb_frame.pFrameData)
        rgb_np = np.ctypeslib.as_array(buf).reshape(rgb_frame.height, rgb_frame.width, 3)
        print("RGB frame shape:", rgb_np.shape)

# --- Stop streaming and cleanup ---
cam.Ps2_StopStream(handle)
cam.Ps2_CloseDevice(handle)
