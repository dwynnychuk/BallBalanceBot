import cv2
import numpy as np

class Camera:
    def __init__(self):
        # import picamera2 only if used on raspberry pi
        try:
            from picamera2 import Picamera2
            self.PiCameraAvailable = True
        except:
            self.PiCameraAvailable = False


    def _get_camera(self):
        if self.PiCameraAvailable:
            # Rapsberry Pi
            picam2 = Picamera2()
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
            cv2.imshow("camera", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            
        cv2.destroyAllWindows()