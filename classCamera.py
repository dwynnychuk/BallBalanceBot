import cv2 as cv
import os
import numpy as np
import threading
import time
from logger import get_logger
from dataclasses import dataclass
from typing import Optional, Tuple, List, Generator

logger = get_logger(__name__)

@dataclass
class CalibrationData:
    """Dataclass to hold calibration data"""
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray

@dataclass
class CameraConfig:
    """Camera configuration parameters"""
    resolution: Tuple[int, int] = (1280, 720)
    downscale_factor: float = 2.0
    target_fps: int = 60
    buffer_size: int = 1

@dataclass
class BallDetectionConfig:
    """Ball Detection Parameters"""
    hsv_lower: np.ndarray = None
    hsv_upper: np.ndarray = None
    morphology_kernal_size: Tuple[int, int] = (5,5)
    min_contour_area: int = 10000
    min_radius: int = 50
    max_radius: int = 400
    
    def __post_init__(self):
        if self.hsv_lower is None:
            self.hsv_lower = np.array([10, 50, 5])
        if self.hsv_upper is None:
            self.hsv_upper = np.array([40, 255, 100])
        
@dataclass 
class BallPosition:
    """Paramters defining ball position and timing"""
    x: float
    y: float
    radius: float
    timestamp: float
    
    def age(self) -> float:
        return time.perf_counter() - self.timestamp
    
    def to_list(self) -> List[float]:
        return [self.x, self.y, self.radius]

class Camera:
    def __init__(
        self,
        camera_config: Optional[CameraConfig] = None,
        detection_config: Optional[BallDetectionConfig] = None,
        enable_visualization: bool = False
    ):
        """
        Main camera interfact for ball detection in ball balancing robot
        
        Args:
            camera_config (Optional[CameraConfig], optional): _description_. Defaults to None.
        """
        self.camera_config = camera_config if camera_config is not None else CameraConfig()
        self.detection_config = detection_config if detection_config is not None else BallDetectionConfig()
        self.enable_visualization = enable_visualization
        
        # Derived Values
        self.small_frame_size = (
            int(self.camera_config.resolution[0] / self.camera_config.downscale_factor),
            int(self.camera_config.resolution[1] / self.camera_config.downscale_factor)
        )
        
        # Camera morphology
        self.kernel = cv.getStructuringElement(
            cv.MORPH_ELLIPSE, 
            self.detection_config.morphology_kernal_size
            )
        
        # Threading safety
        self._lock = threading.Lock()
        self._latest_frame = None
        self._frame_timestamp = None
        self._latest_ball = None
        
        # Threading Control
        self._thread: Optional[threading.Thread] = None
        self._running: bool = False 
        
        # Statistics
        self._frame_count = 0
        self._detection_count = 0
        self._stats_start_time = time.perf_counter()
        
        # Setup functions
        self._setup_camera()
            
    def _setup_camera(self) -> None:
        """Setup Camera and load in calibrations."""
        try:
            from picamera2 import Picamera2
            self._is_pi_camera = True
            self._picam2 = Picamera2()
            logger.info("Using Raspberry PI Camera")
        except:
            self._is_pi_camera = False
            logger.info("Using Webcam")

        self._calibration = self._load_calibration_data()
            
    def _load_calibration_data(self) -> Optional[CalibrationData]:
        """Load calibration data for the camera type being used

        Returns:
            Optional[CalibrationData]: Camera Calibration data
        """
        if self._is_pi_camera:
            calib_path = "calibration/pi_camera_calibration.npz"
            if not os.path.exists(calib_path):
                logger.warning(f"Pi camera calibration not available: File not found: {calib_path}")
                return None
            data = np.load(calib_path)
            return CalibrationData(
                camera_matrix=data["camera_matrix"],
                distortion_coefficients=data["distortion_coefficients"]
            )
        else:
            # TODO -> Move this to a file
            return CalibrationData(
                camera_matrix = np.array([
                    [1.41705209e+03, 0.00000000e+00, 9.60026636e+02],
                    [0.00000000e+00, 1.41704256e+03, 5.45224992e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
                    ]),
                distortion_coefficients = np.array(
                    [-8.49216396e-03,3.08034351e-01, -3.88908240e-03, 1.96616800e-04, -8.78991630e-01]
                    )
            )

    def _get_camera_frames(self) -> Generator[Tuple[np.ndarray, float], None, None]:
        """wrapper function to yield frames from proper source

        Yields:
            Tuple of (frame, timestamp)
        """
        if self._is_pi_camera:
            yield from self._get_pi_camera_frames()
        else:
            yield from self._get_webcam_frames()
    
    def _get_pi_camera_frames(self) -> Generator[Tuple[np.ndarray, float], None, None]:
        """Get frames using raspberry pi camera

        Yields:
            Tuple of (frame, timestamp)
        """
        config = self._picam2.create_video_configuration(
            main={"size": self.camera_config.resolution}, 
            buffer_count = self.camera_config.buffer_size
            )
        self._picam2.configure(config)
        self._picam2.start()
        
        try:
            while self._running:
                frame_rgb = self._picam2.capture_array()
                frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
                yield frame_bgr, time.perf_counter()
        finally:
            self._picam2.stop()
    
    def _get_webcam_frames(self) -> Generator[Tuple[np.ndarray, float], None, None]:
        """Get frames using webcam

        Yields:
            Tuple of (frame, timestamp)
        """
        cap = cv.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("Cannot open webcam")
        
        cap.set(cv.CAP_PROP_BUFFERSIZE, self.camera_config.buffer_size)
        cap.set(cv.CAP_PROP_FPS, self.camera_config.target_fps)
        
        try:
            while self._running:
                ret, frame = cap.read()
                if not ret:
                    logger.error("Failed to read frame from webcam")
                    break
                yield frame, time.perf_counter()
        finally:
            cap.release()

    def _capture_loop(self):
        """Main capture loop to generate frames inside background _thread"""
        try:
            for frame, timestamp in self._get_camera_frames():
                if not self._running:
                    break
                
                self._frame_count += 1
                
                processed = self._process_frame(frame)
                ball = self._detect_ball(processed, frame)                
                
                with self._lock:
                    self._latest_frame = frame
                    self._frame_timestamp = timestamp
                
                    if ball is not None:
                        self._latest_ball = ball
                        self._detection_count += 1
                
                # log every 100 frames
                if self._frame_count % 100 == 0:
                    self._log_statistics()
                    
        except Exception as e:
            logger.error(f"Camera _thread crashed: {e}")
        finally:
            cv.destroyAllWindows()
            logger.info("Camera frame stopped")

    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        """"""
        small_frame = cv.resize(frame, self.small_frame_size)
        
        if self._calibration is not None:
            small_frame = self._undistort_frame(small_frame)
        
        # Threshold by color
        hsv = cv.cvtColor(small_frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(
            hsv,
            self.detection_config.hsv_lower, 
            self.detection_config.hsv_upper
            )
        
        # Morphology operations
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel)

        return mask

    def _undistort_frame(self, frame: np.ndarray) -> np.ndarray:
        """Undistort single frame with camera calibration matrix

        Args:
            frame (np.ndarray): raw frame

        Returns:
            np.ndarray: undistorted frame
        """
        scale_x = self.small_frame_size[0] / self.camera_config.resolution[0]
        scale_y = self.small_frame_size[1] / self.camera_config.resolution[1]
        
        scaled_cam_matrix = self._calibration.camera_matrix.copy()
        scaled_cam_matrix[0, 0] *= scale_x  # fx
        scaled_cam_matrix[1, 1] *= scale_y  # fy
        scaled_cam_matrix[0, 2] *= scale_x  # cx
        scaled_cam_matrix[1, 2] *= scale_y  # cy
        
        return cv.undistort(
            frame, 
            scaled_cam_matrix, 
            self._calibration.distortion_coefficients
            )

    def _detect_ball(
        self, 
        mask: np.ndarray, 
        original_frame = None
        ) -> Optional[BallPosition]:
        """_summary_

        Args:
            mask (np.ndarray): _description_
            original_frame (_type_, optional): _description_. Defaults to None.

        Returns:
            Optional[BallPosition]: _description_
        """
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv.contourArea(contour)
            
            # Adjust area threshold based on scaling
            min_area = self.detection_config.min_contour_area / (
                self.camera_config.downscale_factor ** 2
            )
            
            if area < min_area:
                continue
            
            # Get circle parameters
            (x,y), radius = cv.minEnclosingCircle(contour)
            
            # Scale back to original coordinates
            x *= self.camera_config.downscale_factor
            y *= self.camera_config.downscale_factor
            radius *= self.camera_config.downscale_factor
            
            # Check radius within bounds
            if not (self.detection_config.min_radius < radius < self.detection_config.max_radius):
                continue
            
            # Visualize if enabled
            if self.enable_visualization:
                center = (int(x), int(y))
                cv.circle(original_frame, center, int(radius), (0, 0, 255), 3)
                cv.circle(original_frame, center, 5, (255,0,0), -1)  # draw point at middle of ball
                
            # Log periodically
            if self._frame_count % 100 == 0:
                logger.debug(f"Ball Detected at:  ({x:.1f}, {y:.1f}), r = {radius:.1f}")
                    
                return BallPosition(
                    x = x,
                    y = y,
                    radius = radius,
                    timestamp = time.perf_counter()
                )
                
        return None

    def start(self):
        """Start camera capture thread"""
        if self._running:
            logger.warning("Camera already running")
            return
        
        self._running = True
        self._thread = threading.Thread(
            None,
            target = self._capture_loop,
            daemon=True,
            name = "CameraThread"
            )
        self._thread.start()
        logger.debug("Camera Thread Started")
        
    def stop(self):
        """Stop camera capture thread"""
        if not self._running:
            logger.debug("Camera thread not running, don't need to stop")
            return
        
        logger.info("Stopping camera thread...")
        self._running = False
        
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                logger.warning("Camera thread did not stop when asked")
        
        logger.info("Camera thread stopped")

    def _log_statistics(self) -> None:
        """Log camera performance"""
        elapsed = time.perf_counter() - self_stats_start_time
        fps = 100 / elapsed
        detection_rate = (self._detection_count / self._frame_count) * 100
        
        logger.info(
            f"Camera: {fps:.1f} FPS, {detection_rate:.1f}% detection rate "
            f"({self._detection_count}/{self._frame_count} frames)"
            )
        
        self_stats_start_time = time.perf_counter()

    def get_ball_position(self):
        return self._latest_ball

    def _adjust_ball_coordinate_frame(self, ball: list) -> list:
        x_c = int(self.camera_config.resolution[0]/2)
        y_c = int(self.camera_config.resolution[1]/2)
        x_cam = ball[0] - x_c
        y_cam = ball[1] - y_c
        x_cad = y_cam
        y_cad = -x_cam
        #logger.debug(f"X,Y Adjusted Ball Coords: [{x_cad}, {y_cad}]")
        return [x_cad, y_cad, ball[2]]    
    
    @property
    def frame_age(self):
        """How old is the latest frame?"""
        if self._frame_timestamp is None:
            return None
        return time.perf_counter() - self._frame_timestamp
    
    @property
    def ball_age(self) -> Optional[float]:
        """How old is the latest ball detection"""
        ball = self.get_ball_position()
        if ball is None:
            return None
        return ball.age()    

if __name__ == "__main__":
    cam = Camera()
    cam.start()
    try:
        while True:
            frame = cam._latest_frame
            if frame is not None:
                cv.imshow("frame", cam._latest_frame)
                if cam.get_ball_position():
                    pass
            if cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        cam.stop()
        cv.destroyAllWindows()
            
