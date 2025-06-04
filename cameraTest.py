import cv2
import sys
import platform
import numpy as np

# import picamera2 only if used on raspberry pi
try:
    from picamera2 import Picamera2
    PiCameraAvailable = True
except:
    PiCameraAvailable = False

def get_camera():
    if PiCameraAvailable:
        # Rapsberry Pi
        picam2 = Picamera2()
        print(type(picam2.preview_configuration))
        config = picam2.preview_configuration(main={"format": "RGB888"})
        print("debug")
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

def main():
    for frame in get_camera():
        cv2.imshow("camera", frame)
        if cv2.waitKey(1) and 0xFF == 27:
            break
        
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()