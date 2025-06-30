import cv2
import numpy as np
from logger import get_logger

logger = get_logger(__name__)

class Camera:
    def __init__(self):
        # import picamera2 only if used on raspberry pi
        self.hsv_orange_lower = np.array([2, 120, 70])      # need to tune after cad complete
        self.hsv_orange_upper = np.array([25, 255, 255])    # need to tune after cad 
        self.kernel_shape = (21,21)
        self.contour_area_threshold = 10000
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,self.kernel_shape)
        try:
            from picamera2 import Picamera2
            self.PiCameraAvailable = True
            self.picam2 = Picamera2()

        except:
            self.PiCameraAvailable = False


    def _get_camera(self):
        if self.PiCameraAvailable:
            # Rapsberry Pi
            picam2 = self.picam2
            config = picam2.preview_configuration
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

    def show_camera(self):
        for frame in self._get_camera():
            # Pre-processing
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,self.hsv_orange_lower, self.hsv_orange_upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            #print("HSV at center:", hsv[hsv.shape[0]//2, hsv.shape[1]//2])
            
            # Detect Blobs to find contour of ball
            contours, _ =cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.contour_area_threshold:
                    print(area)
            cv2.imshow("masked_open_closed_image", mask)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            
        cv2.destroyAllWindows()
        
if __name__ == "__main__":
    cam = Camera()
    cam.show_camera()