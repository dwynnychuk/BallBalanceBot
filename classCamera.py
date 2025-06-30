import cv2
import numpy as np
import threading
from logger import get_logger
import time

logger = get_logger(__name__)

class Camera:
    def __init__(self):
        # import picamera2 only if used on raspberry pi
        self.hsv_orange_lower = np.array([2, 120, 70])      # need to tune after cad complete
        self.hsv_orange_upper = np.array([25, 255, 255])    # need to tune after cad 
        self.kernel_shape = (21,21)
        self.contour_area_threshold = 10000
        self.radius_threshold = [50, 400]
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,self.kernel_shape)
        self.latest_ball_pos = None
        self.t0 = time.time()
        self.tn1 = time.time()-1
        self.latest_frame = None
        try:
            from picamera2 import Picamera2
            self.PiCameraAvailable = True
            self.picam2 = Picamera2()

        except:
            self.PiCameraAvailable = False

    @property
    def delta_t(self):
        return self.t0-self.tn1

    def _get_camera(self):
        if self.PiCameraAvailable:
            # Rapsberry Pi
            picam2 = self.picam2
            
            config = picam2.create_preview_configuration(main={"size": (800, 800)},
                                                         raw={"size": (3280,2464)})
            picam2.set_controls({"ScalerCrop": None})
            picam2.configure(config)
            picam2.start()

            while True:
                frame_rgb = picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                yield frame_bgr
        else:
            # Mac
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                raise RuntimeError("Cannot Open Webcam")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to get frame")
                    break
                yield frame
            cap.release()

    def _capture_camera(self):
        for frame in self._get_camera():
            self.latest_frame = frame
            ball = self._process_image(frame)
            self.latest_ball_pos = ball
            
        cv2.destroyAllWindows()

    def start(self):
        thread = threading.Thread(None,target=self._capture_camera, daemon=True)
        thread.start()

    def _process_image(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,self.hsv_orange_lower, self.hsv_orange_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        #print("HSV at center:", hsv[hsv.shape[0]//2, hsv.shape[1]//2])
        
        # Detect Blobs to find contour of ball
        contours, _ =cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.contour_area_threshold:
                (x,y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                if self.radius_threshold[0] < radius < self.radius_threshold[1]:
                    cv2.circle(frame, center, radius,(0, 0, 255),3)
                    cv2.circle(frame, center, 5, (255,0,0),10)  # draw point at middle of ball
                    ball = [center[0], center[1], radius]
                    self.tn1 = self.t0
                    self.t0 = time.time()
                    return ball
        return ball

    def get_ball_position(self):
        return self.latest_ball_pos

if __name__ == "__main__":
    cam = Camera()
    cam.start()
    
    while True:
        frame = cam.latest_frame
        if frame is not None:
            cv2.imshow("frame", cam.latest_frame)
            if cam.get_ball_position():
                print(f"Ball POS: {cam.get_ball_position()}, TIME: {cam.delta_t}")
        if cv2.waitKey(1) & 0xFF == 27:
            break
            
