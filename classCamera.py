import cv2 as cv
import os
import numpy as np
import threading
from logger import get_logger
import time

logger = get_logger(__name__)

class Camera:
    def __init__(self):
        # import picamera2 only if used on raspberry pi
        self.hsv_lower = np.array([10, 50, 5])      # need to tune after cad complete
        self.hsv_upper = np.array([40, 255, 100])    # need to tune after cad 
        self.camera_fov = (1280, 720)
        self.scale = 4.0
        self.small_frame_size = (int(self.camera_fov[0]/self.scale), int(self.camera_fov[1]/self.scale))
        self.kernel_shape = (5,5)
        self.contour_area_threshold = 10000
        self.radius_threshold = [50, 400]
        self.kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, self.kernel_shape)
        self.latest_ball_pos = None
        self.latest_ball_timestamp = None
        self.frame_timestamp = None
        self.t0 = time.perf_counter()  
        self.tn1 = time.perf_counter() - 1
        self.latest_frame = None
        self.running = True
        self.frame_count = 0
        self.detection_count = 0
        self.fps_start_time = time.perf_counter()
        
        try:
            from picamera2 import Picamera2
            self.PiCameraAvailable = True
            self.picam2 = Picamera2()

        except:
            self.PiCameraAvailable = False
            
    def _read_calibration_data(self, camType):
        if camType == "MAC":
            dist = np.array([-8.49216396e-03,3.08034351e-01, -3.88908240e-03, 1.96616800e-04, -8.78991630e-01])
            cam = np.array([[1.41705209e+03, 0.00000000e+00, 9.60026636e+02],
                            [0.00000000e+00, 1.41704256e+03, 5.45224992e+02],
                            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
            
        elif camType == "PI":
            calib = "calibration/pi_camera_calibration.npz"
            if not os.path.exists(calib):
                logger.warning("Pi camera calibration not available: File not found")
                return None, None
            data = np.load(calib)
            dist, cam = data["distortion_coefficients"], data["camera_matrix"]
            
        return dist, cam

    def _get_camera(self):
        if self.PiCameraAvailable:
            # Rapsberry Pi
            self.distCoefs, self.camMat = self._read_calibration_data("PI")
            
            picam2 = self.picam2
            
            config = picam2.create_video_configuration(main={"size": self.camera_fov}, 
                                                       buffer_count=2)

            picam2.configure(config)
            picam2.start()

            try:
                while self.running:
                    frame_rgb = picam2.capture_array()
                    frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
                    yield frame_bgr, time.perf_counter()
            finally:
                picam2.stop()
        else:
            # Mac
            self.distCoefs, self.camMat = self._read_calibration_data("MAC")
            cap = cv.VideoCapture(0)
            if not cap.isOpened():
                logger.error("Cannot Open Webcam")
                raise RuntimeError("Cannot Open Webcam")
            
            cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv.CAP_PROP_FPS, 60)
            
            try:
                while self.running:
                    ret, frame = cap.read()
                    if not ret:
                        logger.error("Failed to get frame")
                        break
                    yield frame, time.perf_counter()
            finally:
                cap.release()

    def _capture_camera(self):
        for frame, timestamp in self._get_camera():
            if not self.running:
                break
            self.frame_count += 1
            self.latest_frame = frame
            self.frame_timestamp = timestamp
            
            processed = self._process_image(frame)
            bal_pos = self._find_ball(processed, unprocessed=frame.copy())
            
            if bal_pos is not None:
                self.latest_ball_pos = bal_pos
                self.latest_ball_timestamp = time.perf_counter()
                self.detection_count += 1
            
            # log every 100 frames
            if self.frame_count % 100 == 0:
                elapsed = time.perf_counter() - self.fps_start_time
                fps = 1.0 / elapsed
                logger.info(f"Camera FPS: {fps:.1f}")
                detection_rate = (self.detection_count / self.frame_count) * 100
                logger.info(f"Camera: {fps:.1f} FPS, {detection_rate:.1f}% detection rate")
                self.fps_start_time = time.perf_counter()
            
        cv.destroyAllWindows()

    @property
    def delta_t(self):
        return self.t0 - self.tn1
    
    @property
    def frame_age(self):
        """How old is the latest frame?"""
        if self.frame_timestamp is None:
            return None
        return time.perf_counter() - self.frame_timestamp
    
    @property
    def ball_age(self):
        """How old is the latest ball detection?"""
        if self.latest_ball_timestamp is None:
            return None
        return time.perf_counter() - self.latest_ball_timestamp

    def start(self):
        self.thread = threading.Thread(None,target=self._capture_camera, daemon=True)
        self.thread.start()
        logger.debug("Camera Thread Started")
        
    def stop(self):
        self.running = False
        if self.PiCameraAvailable:
            self.picam2.stop()

    def _process_image(self, frame):
        if self.camMat is not None and self.distCoefs is not None:
            # Scale camera matrix for downsampled image
            scale_x = self.small_frame_size[0] / self.camera_fov[0]
            scale_y = self.small_frame_size[1] / self.camera_fov[1]
            scaled_cam = self.camMat.copy()
            scaled_cam[0, 0] *= scale_x  # fx
            scaled_cam[1, 1] *= scale_y  # fy
            scaled_cam[0, 2] *= scale_x  # cx
            scaled_cam[1, 2] *= scale_y  # cy
            
            small_frame = cv.undistort(small_frame, scaled_cam, self.distCoefs)
        
        small_frame = cv.resize(frame, self.small_frame_size)
        hsv = cv.cvtColor(small_frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv,self.hsv_lower, self.hsv_upper)
        
        # reduce morphology operations
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel)
        #logger.debug("HSV at center:", hsv[hsv.shape[0]//2, hsv.shape[1]//2])

        return mask
    
    def _find_ball(self, frame, unprocessed = None):
        """Detect Blobs to find contour of ball
            OUTPUT: Ball [x, y, radius]"""
        contours, _ = cv.findContours(frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        ball = None
        
        for contour in contours:
            area = cv.contourArea(contour)
            # Adjust area threshold based on scaling
            if area > (self.contour_area_threshold / (self.scale**2)):
                (x,y), radius = cv.minEnclosingCircle(contour)
                
                # scale to original coordinates
                x *= self.scale
                y *= self.scale
                radius *= self.scale
                
                center = (int(x), int(y))
                radius = int(radius)
                
                if self.radius_threshold[0] < radius < self.radius_threshold[1]:
                    #if unprocessed is not None:
                        #cv.drawContours(unprocessed,contour, -1, (255,255,0),4)
                    #    cv.circle(unprocessed, center, radius,(0, 0, 255),3)
                    #    cv.circle(unprocessed, center, 5, (255,0,0),10)  # draw point at middle of ball
                    ball = [center[0], center[1], radius]
                    self.tn1 = self.t0
                    self.t0 = time.perf_counter()
                    
                    if self.frame_count % 30 == 0:
                        logger.debug(f"Ball Pos: {ball}, Time: {self.delta_t*1000:.1f}")
                        
                    return ball
        return ball 

    def get_ball_position(self):
        return self.latest_ball_pos

    def _adjust_ball_coordinate_frame(self, ball: list) -> list:
        x_c = int(self.camera_fov[0]/2)
        y_c = int(self.camera_fov[1]/2)
        x_cam = ball[0] - x_c
        y_cam = ball[1] - y_c
        x_cad = y_cam
        y_cad = -x_cam
        #logger.debug(f"X,Y Adjusted Ball Coords: [{x_cad}, {y_cad}]")
        return [x_cad, y_cad, ball[2]]        

if __name__ == "__main__":
    cam = Camera()
    cam.start()
    try:
        while True:
            frame = cam.latest_frame
            if frame is not None:
                cv.imshow("frame", cam.latest_frame)
                if cam.get_ball_position():
                    pass
            if cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        cam.stop()
        cv.destroyAllWindows()
            
