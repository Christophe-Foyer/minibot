"""
Pico Zense DCAM710 Camera Module - Fixed Version

A corrected Python wrapper for the Pico Zense DCAM710 depth camera.
Based on the actual Vzense API structure.

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
import os
from pathlib import Path
import sys
from typing import Optional, Tuple, Union

try:
    # Import the Vzense API - using the actual class structure
    sys.path.append(os.path.join(Path(__file__).parent, "BaseSDK_python_wrapper"))
    from DCAM710.API.Vzense_api_710 import VzenseTofCam
    from DCAM710.API.Vzense_define_710 import *
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
    and clean up resources. Based on the actual Vzense API structure.
    """
    
    def __init__(self, device_index: int = 0, timeout_ms: int = 33):
        """
        Initialize the DCAM710 camera interface.
        
        Args:
            device_index: Index of the device to use (0 for first camera)
            timeout_ms: Wait time for frame reading in milliseconds
        """
        if not VZENSE_AVAILABLE:
            raise ImportError("Vzense API not available. Please install the required SDK.")
        
        self.device_index = device_index
        self.timeout_ms = timeout_ms
        self.vzense_cam = None
        self.is_initialized = False
        self.is_streaming = False
        
        # Camera properties
        self.depth_width = 640
        self.depth_height = 480
        self.color_width = 1280
        self.color_height = 720
    
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
            # Create the camera instance (this initializes the SDK)
            self.vzense_cam = VzenseTofCam()
            
            # Get device count
            device_count = self.vzense_cam.Ps2_GetDeviceCount()
            if device_count <= self.device_index:
                logger.error(f"Device index {self.device_index} not found. Available devices: {device_count}")
                return False
            
            logger.info(f"Found {device_count} device(s)")
            
            # Get device info
            ret, device_info = self.vzense_cam.Ps2_GetDeviceInfo(self.device_index)
            if ret != 0:
                logger.error(f"Failed to get device info. Error code: {ret}")
                return False
                
            logger.info(f"Opening device: {device_info.alias}")
            
            # Open the device using URI
            ret = self.vzense_cam.Ps2_OpenDevice(device_info.uri)
            if ret != 0:
                logger.error(f"Failed to open device. Error code: {ret}")
                return False
            
            # Set up camera parameters
            self._configure_camera()
            
            self.is_initialized = True
            logger.info("Camera initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during initialization: {e}")
            return False
    
    def _configure_camera(self):
        """Configure camera settings"""
        try:
            # Set data mode (depth + RGB)
            ret = self.vzense_cam.Ps2_SetDataMode(PsDataMode.PsDepthAndRGB_30)
            if ret != 0:
                logger.warning(f"Failed to set data mode: {ret}")
            
            # Set RGB resolution
            ret = self.vzense_cam.Ps2_SetRGBResolution(PsResolution.PsRGB_Resolution_1280_720)
            if ret != 0:
                logger.warning(f"Failed to set RGB resolution: {ret}")
            else:
                self.color_width = 1280
                self.color_height = 720
            
            # Set color pixel format to BGR
            ret = self.vzense_cam.Ps2_SetColorPixelFormat(PsPixelFormat.PsPixelFormatBGR888)
            if ret != 0:
                logger.warning(f"Failed to set color format: {ret}")
            
            # Enable RGB and depth frames
            self.vzense_cam.Ps2_SetRgbFrameEnabled(True)
            self.vzense_cam.Ps2_SetDepthFrameEnabled(True)
            
            # Set frame reading timeout
            self.vzense_cam.Ps2_SetWaitTimeOfReadNextFrame(self.timeout_ms)
            
            logger.info(f"Camera configured - Depth: {self.depth_width}x{self.depth_height}, RGB: {self.color_width}x{self.color_height}")
            
        except Exception as e:
            logger.warning(f"Error during camera configuration: {e}")
    
    def start(self) -> bool:
        """
        Start camera streaming.
        
        Returns:
            bool: True if streaming started successfully, False otherwise
        """
        if not self.is_initialized:
            if not self.initialize():
                return False
        
        try:
            # Start streaming
            ret = self.vzense_cam.Ps2_StartStream()
            if ret != 0:
                logger.error(f"Failed to start stream. Error code: {ret}")
                return False
            
            self.is_streaming = True
            logger.info("Streaming started successfully")
            
            # Wait for streams to stabilize
            time.sleep(0.5)
            
            return True
            
        except Exception as e:
            logger.error(f"Error starting stream: {e}")
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
            # Read next frame - this gets frame ready status
            ret, frame_ready = self.vzense_cam.Ps2_ReadNextFrame()
            if ret != 0:
                logger.debug(f"Frame not ready: {ret}")
                return None, None
            
            # Get depth frame if available
            if hasattr(frame_ready, 'depth') and frame_ready.depth:
                ret, depth_frame = self.vzense_cam.Ps2_GetFrame(PsFrameType.PsDepthFrame)
                if ret == 0 and depth_frame.pFrameData:
                    depth_data = np.frombuffer(depth_frame.pFrameData, dtype=np.uint16)
                    if return_raw:
                        depth_image = depth_data
                    else:
                        depth_image = depth_data.reshape((depth_frame.height, depth_frame.width))
            
            # Get color frame if available  
            if hasattr(frame_ready, 'rgb') and frame_ready.rgb:
                ret, color_frame = self.vzense_cam.Ps2_GetFrame(PsFrameType.PsRGBFrame)
                if ret == 0 and color_frame.pFrameData:
                    color_data = np.frombuffer(color_frame.pFrameData, dtype=np.uint8)
                    if return_raw:
                        rgb_image = color_data
                    else:
                        # BGR format from camera
                        rgb_image = color_data.reshape((color_frame.height, color_frame.width, 3))
            
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
            # Read next frame
            ret, frame_ready = self.vzense_cam.Ps2_ReadNextFrame()
            if ret != 0:
                return None
            
            if hasattr(frame_ready, 'depth') and frame_ready.depth:
                ret, depth_frame = self.vzense_cam.Ps2_GetFrame(PsFrameType.PsDepthFrame)
                if ret == 0 and depth_frame.pFrameData:
                    depth_data = np.frombuffer(depth_frame.pFrameData, dtype=np.uint16)
                    return depth_data.reshape((depth_frame.height, depth_frame.width))
                    
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
            # Read next frame
            ret, frame_ready = self.vzense_cam.Ps2_ReadNextFrame()
            if ret != 0:
                return None
                
            if hasattr(frame_ready, 'rgb') and frame_ready.rgb:
                ret, color_frame = self.vzense_cam.Ps2_GetFrame(PsFrameType.PsRGBFrame)
                if ret == 0 and color_frame.pFrameData:
                    color_data = np.frombuffer(color_frame.pFrameData, dtype=np.uint8)
                    return color_data.reshape((color_frame.height, color_frame.width, 3))
                    
        except Exception as e:
            logger.error(f"Error capturing color frame: {e}")
        
        return None
    
    def stop(self):
        """Stop streams and cleanup resources"""
        try:
            if self.is_streaming and self.vzense_cam:
                ret = self.vzense_cam.Ps2_StopStream()
                if ret == 0:
                    logger.info("Stream stopped")
                else:
                    logger.warning(f"Error stopping stream: {ret}")
                self.is_streaming = False
            
            if self.vzense_cam:
                ret = self.vzense_cam.Ps2_CloseDevice()
                if ret == 0:
                    logger.info("Device closed")
                else:
                    logger.warning(f"Error closing device: {ret}")
                
                # The destructor will call Ps2_Shutdown()
                self.vzense_cam = None
                self.is_initialized = False
                logger.info("SDK cleanup completed")
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
    
    def get_camera_info(self) -> dict:
        """
        Get camera information and parameters.
        
        Returns:
            dict: Camera information
        """
        info = {
            'depth_resolution': (self.depth_width, self.depth_height),
            'color_resolution': (self.color_width, self.color_height),
            'is_initialized': self.is_initialized,
            'is_streaming': self.is_streaming,
            'device_index': self.device_index
        }
        
        # Add additional info if camera is initialized
        if self.is_initialized and self.vzense_cam:
            try:
                ret, sn = self.vzense_cam.Ps2_GetSerialNumber()
                if ret == 0:
                    info['serial_number'] = sn.decode() if isinstance(sn, bytes) else str(sn)
                
                ret, fw = self.vzense_cam.Ps2_GetFirmwareVersionNumber()
                if ret == 0:
                    info['firmware_version'] = fw.decode() if isinstance(fw, bytes) else str(fw)
                    
                ret, sdk_version = self.vzense_cam.Ps2_GetSDKVersion()
                if ret == 0:
                    info['sdk_version'] = sdk_version.decode() if isinstance(sdk_version, bytes) else str(sdk_version)
                    
            except Exception as e:
                logger.debug(f"Could not get additional camera info: {e}")
        
        return info
    
    def set_depth_range(self, depth_range: 'PsDepthRange' = None):
        """
        Set the depth range for the camera.
        
        Args:
            depth_range: PsDepthRange enum value (PsNearRange, PsMidRange, PsFarRange, etc.)
        """
        if not self.is_initialized:
            logger.warning("Camera not initialized")
            return False
            
        if depth_range is None:
            depth_range = PsDepthRange.PsNearRange
            
        try:
            ret = self.vzense_cam.Ps2_SetDepthRange(depth_range)
            if ret == 0:
                logger.info(f"Depth range set successfully")
                return True
            else:
                logger.warning(f"Failed to set depth range: {ret}")
                return False
        except Exception as e:
            logger.error(f"Error setting depth range: {e}")
            return False
    
    def get_measuring_range(self, depth_range: 'PsDepthRange' = None) -> dict:
        """
        Get measuring range information for specified depth range.
        
        Returns:
            dict: Range information with max_depth, min_effective, max_effective
        """
        if not self.is_initialized:
            logger.warning("Camera not initialized")
            return {}
            
        if depth_range is None:
            depth_range = PsDepthRange.PsNearRange
            
        try:
            ret, max_depth, min_eff, max_eff = self.vzense_cam.Ps2_GetMeasuringRange(depth_range)
            if ret == 0:
                return {
                    'max_depth': max_depth,
                    'min_effective': min_eff,
                    'max_effective': max_eff,
                    'range_type': str(depth_range)
                }
            else:
                logger.warning(f"Failed to get measuring range: {ret}")
                return {}
        except Exception as e:
            logger.error(f"Error getting measuring range: {e}")
            return {}


# Convenience function for quick testing
def test_camera():
    """Simple test function to verify camera functionality"""
    try:
        with DCAM710Camera() as camera:
            if not camera.start():
                print("Failed to start camera")
                return
            
            print("Camera started successfully!")
            camera_info = camera.get_camera_info()
            print(f"Camera info: {camera_info}")
            
            # Test measuring range
            range_info = camera.get_measuring_range()
            if range_info:
                print(f"Measuring range: {range_info}")
            
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
