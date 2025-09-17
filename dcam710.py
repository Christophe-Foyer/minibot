"""
Pico Zense DCAM710 Camera Module

A simple Python wrapper for the Pico Zense DCAM710 depth camera.
Provides easy access to RGB and depth images.

Requirements:
- Vzense BaseSDK installed
- Vzense Python wrapper (BaseSDK_python_wrapper)
- NumPy
- OpenCV (optional, for display/processing)

Usage:
    from dcam710_camera import DCAM710Camera
    
    camera = DCAM710Camera()
    camera.start()
    
    try:
        while True:
            rgb_image, depth_image = camera.get_frames()
            if rgb_image is not None and depth_image is not None:
                # Process your images here
                print(f"RGB shape: {rgb_image.shape}, Depth shape: {depth_image.shape}")
    finally:
        camera.stop()
        
    # Or use context manager:
    with DCAM710Camera() as camera:
        camera.start()
        rgb, depth = camera.get_frames()
"""

import numpy as np
import time
import logging
from typing import Optional, Tuple, Union

try:
    # Import the Vzense API
    # BaseSDK_python_wrapper/DCAM710/API/Vzense_api_710.py
    from BaseSDK_python_wrapper.DCAM710.API.Vzense_api_710 import *  # TODO: Get rid of star import
    VZENSE_AVAILABLE = True
except ImportError:
    print("Warning: Vzense API not found. Please install the BaseSDK_python_wrapper.")
    print("Download from: https://github.com/Vzense/BaseSDK_python_wrapper")
    VZENSE_AVAILABLE = False

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DCAM710Camera:
    """
    A wrapper class for the Pico Zense DCAM710 depth camera.
    
    Provides simple methods to initialize the camera, capture frames,
    and clean up resources.
    """
    
    def __init__(self, device_index: int = 0, timeout_ms: int = 1000):
        """
        Initialize the DCAM710 camera interface.
        
        Args:
            device_index: Index of the device to use (0 for first camera)
            timeout_ms: Timeout for frame capture in milliseconds
s        """
        if not VZENSE_AVAILABLE:
            raise ImportError("Vzense API not available. Please install the required SDK.")
        
        self.device_index = device_index
        self.timeout_ms = timeout_ms
        self.device_handle = None
        self.is_initialized = False
        self.is_streaming = False
        
        # Camera properties (will be set during initialization)
        self.depth_width = 0
        self.depth_height = 0
        self.color_width = 0
        self.color_height = 0
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup"""
        self.stop()
    
    def initialize(self) -> bool:
        """
        Initialize the camera SDK and open the device.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            # Initialize the SDK
            ret = Ps2_Initialize()
            if ret != 0:
                logger.error(f"Failed to initialize SDK. Error code: {ret}")
                return False
            
            # Get device count
            device_count = Ps2_GetDeviceCount()
            if device_count <= self.device_index:
                logger.error(f"Device index {self.device_index} not found. Available devices: {device_count}")
                return False
            
            logger.info(f"Found {device_count} device(s)")
            
            # Get device info
            device_info = Ps2_GetDeviceInfo(self.device_index)
            logger.info(f"Opening device: {device_info.alias}")
            
            # Open the device
            self.device_handle = Ps2_OpenDevice(device_info.uri)
            if self.device_handle is None:
                logger.error("Failed to open device")
                return False
            
            # Get camera parameters
            self._get_camera_parameters()
            
            self.is_initialized = True
            logger.info("Camera initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during initialization: {e}")
            return False
    
    def _get_camera_parameters(self):
        """Get and store camera parameters"""
        try:
            # Get depth stream info
            depth_info = Ps2_GetStreamInfo(self.device_handle, PsStreamType.PsDepthStream)
            self.depth_width = depth_info.width
            self.depth_height = depth_info.height
            
            # Get color stream info
            color_info = Ps2_GetStreamInfo(self.device_handle, PsStreamType.PsColorStream)
            self.color_width = color_info.width
            self.color_height = color_info.height
            
            logger.info(f"Depth resolution: {self.depth_width}x{self.depth_height}")
            logger.info(f"Color resolution: {self.color_width}x{self.color_height}")
            
        except Exception as e:
            logger.warning(f"Could not get camera parameters: {e}")
            # Set default values
            self.depth_width = 640
            self.depth_height = 480
            self.color_width = 1280
            self.color_height = 720
    
    def start(self) -> bool:
        """
        Start depth and color streams.
        
        Returns:
            bool: True if streams started successfully, False otherwise
        """
        if not self.is_initialized:
            if not self.initialize():
                return False
        
        try:
            # Start depth stream
            ret_depth = Ps2_StartStream(self.device_handle, PsStreamType.PsDepthStream)
            if ret_depth != 0:
                logger.error(f"Failed to start depth stream. Error code: {ret_depth}")
                return False
            
            # Start color stream
            ret_color = Ps2_StartStream(self.device_handle, PsStreamType.PsColorStream)
            if ret_color != 0:
                logger.error(f"Failed to start color stream. Error code: {ret_color}")
                # Stop depth stream if color failed
                Ps2_StopStream(self.device_handle, PsStreamType.PsDepthStream)
                return False
            
            self.is_streaming = True
            logger.info("Streams started successfully")
            
            # Wait a moment for streams to stabilize
            time.sleep(0.5)
            
            return True
            
        except Exception as e:
            logger.error(f"Error starting streams: {e}")
            return False
    
    def get_frames(self, return_raw: bool = False) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Capture RGB and depth frames.
        
        Args:
            return_raw: If True, return raw data without reshaping
            
        Returns:
            Tuple of (rgb_image, depth_image) as numpy arrays.
            Returns (None, None) if capture fails.
        """
        if not self.is_streaming:
            logger.warning("Camera is not streaming. Call start() first.")
            return None, None
        
        rgb_image = None
        depth_image = None
        
        try:
            # Get depth frame
            depth_frame = Ps2_ReadNextFrame(self.device_handle, PsStreamType.PsDepthStream)
            if depth_frame.pFrameData:
                depth_data = np.frombuffer(depth_frame.pFrameData, dtype=np.uint16)
                if return_raw:
                    depth_image = depth_data
                else:
                    depth_image = depth_data.reshape((self.depth_height, self.depth_width))
            
            # Get color frame
            color_frame = Ps2_ReadNextFrame(self.device_handle, PsStreamType.PsColorStream)
            if color_frame.pFrameData:
                color_data = np.frombuffer(color_frame.pFrameData, dtype=np.uint8)
                if return_raw:
                    rgb_image = color_data
                else:
                    # Assuming BGR format (common for many cameras)
                    rgb_image = color_data.reshape((self.color_height, self.color_width, 3))
            
        except Exception as e:
            logger.error(f"Error capturing frames: {e}")
            return None, None
        
        return rgb_image, depth_image
    
    def get_depth_frame(self) -> Optional[np.ndarray]:
        """
        Capture only the depth frame.
        
        Returns:
            np.ndarray: Depth image as uint16 array, or None if capture fails
        """
        if not self.is_streaming:
            logger.warning("Camera is not streaming. Call start() first.")
            return None
        
        try:
            depth_frame = Ps2_ReadNextFrame(self.device_handle, PsStreamType.PsDepthStream)
            if depth_frame.pFrameData:
                depth_data = np.frombuffer(depth_frame.pFrameData, dtype=np.uint16)
                return depth_data.reshape((self.depth_height, self.depth_width))
        except Exception as e:
            logger.error(f"Error capturing depth frame: {e}")
        
        return None
    
    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """
        Capture only the RGB frame.
        
        Returns:
            np.ndarray: RGB/BGR image as uint8 array, or None if capture fails
        """
        if not self.is_streaming:
            logger.warning("Camera is not streaming. Call start() first.")
            return None
        
        try:
            color_frame = Ps2_ReadNextFrame(self.device_handle, PsStreamType.PsColorStream)
            if color_frame.pFrameData:
                color_data = np.frombuffer(color_frame.pFrameData, dtype=np.uint8)
                return color_data.reshape((self.color_height, self.color_width, 3))
        except Exception as e:
            logger.error(f"Error capturing color frame: {e}")
        
        return None
    
    def stop(self):
        """Stop streams and cleanup resources"""
        try:
            if self.is_streaming and self.device_handle:
                Ps2_StopStream(self.device_handle, PsStreamType.PsDepthStream)
                Ps2_StopStream(self.device_handle, PsStreamType.PsColorStream)
                self.is_streaming = False
                logger.info("Streams stopped")
            
            if self.device_handle:
                Ps2_CloseDevice(self.device_handle)
                self.device_handle = None
                logger.info("Device closed")
            
            if self.is_initialized:
                Ps2_Shutdown()
                self.is_initialized = False
                logger.info("SDK shutdown")
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
    
    def get_camera_info(self) -> dict:
        """
        Get camera information and parameters.
        
        Returns:
            dict: Camera information
        """
        return {
            'depth_resolution': (self.depth_width, self.depth_height),
            'color_resolution': (self.color_width, self.color_height),
            'is_initialized': self.is_initialized,
            'is_streaming': self.is_streaming,
            'device_index': self.device_index
        }


# Convenience function for quick testing
def test_camera():
    """Simple test function to verify camera functionality"""
    try:
        with DCAM710Camera() as camera:
            if not camera.start():
                print("Failed to start camera")
                return
            
            print("Camera started successfully!")
            print(f"Camera info: {camera.get_camera_info()}")
            
            # Capture a few frames
            for i in range(5):
                rgb, depth = camera.get_frames()
                if rgb is not None and depth is not None:
                    print(f"Frame {i+1}: RGB shape: {rgb.shape}, Depth shape: {depth.shape}")
                    print(f"  Depth range: {depth.min()} - {depth.max()} mm")
                else:
                    print(f"Frame {i+1}: Failed to capture")
                
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test error: {e}")


if __name__ == "__main__":
    # Run test when script is executed directly
    test_camera()
