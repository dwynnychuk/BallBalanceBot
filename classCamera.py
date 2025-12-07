import cv2 as cv
import numpy as np
import threading
from logger import get_logger
import time

logger = get_logger(__name__)

class Camera:
    def __init__(self):
        # import picamera2 only if used on raspberry pi
        self.hsv_lower = np.array([20, 120, 120])      # need to tune after cad complete
        self.hsv_upper = np.array([30, 255, 255])    # need to tune after cad 
        self.kernel_shape = (11,11)
        self.contour_area_threshold = 10000
        self.radius_threshold = [50, 400]
        self.kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,self.kernel_shape)
        self.latest_ball_pos = None
        self.t0 = time.time()
        self.tn1 = time.time()-1
        self.latest_frame = None
        self.running = True
        
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
            dist = np.zeros((1,5))
            cam = np.zeros((3,3))
            
        return dist, cam

    @property
    def delta_t(self):
        return self.t0-self.tn1

    def _get_camera(self):
        if self.PiCameraAvailable:
            # Rapsberry Pi
            self.distCoefs, self.camMat = self._read_calibration_data("PI")
            
            picam2 = self.picam2
            
            config = picam2.create_preview_configuration(main={"size": (800, 800)},
                                                         raw={"size": (3280,2464)})
            picam2.set_controls({"ScalerCrop": None})
            picam2.configure(config)
            picam2.start()

            try:
                while self.running:
                    frame_rgb = picam2.capture_array()
                    frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
                    yield frame_bgr
            finally:
                picam2.stop()
        else:
            # Mac
            self.distCoefs, self.camMat = self._read_calibration_data("MAC")
            cap = cv.VideoCapture(0)
            if not cap.isOpened():
                logger.error("Cannot Open Webcam")
                raise RuntimeError("Cannot Open Webcam")
            
            try:
                while self.running:
                    ret, frame = cap.read()
                    if not ret:
                        logger.error("Failed to get frame")
                        break
                    yield frame
            finally:
                cap.release()

    def _capture_camera(self):
        for frame in self._get_camera():
            if not self.running:
                break
            self.latest_frame = frame
            processed = self._process_image(frame)
            self.latest_ball_pos = self._find_ball(processed, unprocessed=frame)
            
        cv.destroyAllWindows()

    def start(self):
        self.thread = threading.Thread(None,target=self._capture_camera, daemon=True)
        self.thread.start()
        logger.debug("Camera Thread Started")
        
    def stop(self):
        self.running = False
        if self.PiCameraAvailable:
            self.picam2.stop()

    def _process_image(self, frame):
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv,self.hsv_lower, self.hsv_upper)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel)
        #logger.debug("HSV at center:", hsv[hsv.shape[0]//2, hsv.shape[1]//2])
        return mask
    
    def _find_ball(self, frame, unprocessed = None):
        """Detect Blobs to find contour of ball"""
        contours, _ = cv.findContours(frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        ball = None
        for contour in contours:
            area = cv.contourArea(contour)
            if area > self.contour_area_threshold:
                (x,y), radius = cv.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                if self.radius_threshold[0] < radius < self.radius_threshold[1]:
                    if unprocessed is not None:
                        #cv.drawContours(unprocessed,contour, -1, (255,255,0),4)
                        cv.circle(unprocessed, center, radius,(0, 0, 255),3)
                        cv.circle(unprocessed, center, 5, (255,0,0),10)  # draw point at middle of ball
                    ball = [center[0], center[1], radius]
                    self.tn1 = self.t0
                    self.t0 = time.time()
                    logger.debug(f"Ball Pos: {ball}, Time: {self.delta_t}")
                    return ball
        return ball 

    def get_ball_position(self):
        return self.latest_ball_pos

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
            