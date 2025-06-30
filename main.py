import classCamera
import classControl
import classRobot
import classServo
from logger import get_logger
import time
import cv2

logger = get_logger(__name__)

def main():
    servos = classServo.init_servos(10)
    cam = classCamera.Camera()
    cam.start()
    
    while True:
        frame = cam.latest_frame
        if frame is not None:
            cv2.imshow("frame", cam.latest_frame)
            if cam.get_ball_position():
                print(f"Ball POS: {cam.get_ball_position()}, TIME: {cam.delta_t}")

        if cv2.waitKey(1) & 0xFF == 27:
            break
        cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()