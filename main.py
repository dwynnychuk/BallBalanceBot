import classCamera
import classControl
import classRobot
import classServo
from logger import get_logger
import time
import cv2 as cv

logger = get_logger(__name__)

def main():
    servos = classServo.init_servos(10)
    cam = classCamera.Camera()
    cam.start()
    
    while True:
        frame = cam.latest_frame
        if frame is not None:
            cv.imshow("frame", cam.latest_frame)
            if cam.get_ball_position():
                logger.debug(f"Ball POS: {cam.get_ball_position()}, TIME: {cam.delta_t}")

        if cv.waitKey(1) & 0xFF == 27:
            cam.running = False
            break
        cv.destroyAllWindows()
    
if __name__ == "__main__":
    main()