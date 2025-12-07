import classCamera
import classControl
import classRobot
import classServo
from logger import get_logger
import time
import cv2 as cv

logger = get_logger(__name__)

def main():
    servos: classServo.Servo = classServo.init_servos(3)
    cam = classCamera.Camera()
    cam.start()
    try:
        while True:
            frame = cam.latest_frame
            if frame is not None:
                #cv.imshow("frame", cam.latest_frame)
                if cam.get_ball_position():
                    pass
            if cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        cam.stop()
        cv.destroyAllWindows()
    
if __name__ == "__main__":
    main()