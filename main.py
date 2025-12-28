import classCamera
import classControl
import classRobot
import classServo
from logger import get_logger
import time
import cv2 as cv

logger = get_logger(__name__)

def main():
    # TODO servo init can be cleaned up
    servos: classServo.Servo = classServo.init_servos(3)
    cam = classCamera.Camera()
    pid = classControl.PID()
    robot = classRobot.Robot()
    
    cam.start()
    
    # Initial Conditions
    setpoint = [0,0]
    desired_height = robot.L[0]     # approximation
    
    try:
        while True:
            frame = cam.latest_frame
            ball = cam.get_ball_position()
            
            if frame is not None:
                cv.imshow("frame", cv.resize(frame, (640,480)))
                
            if ball is not None:
                # Convert coordinates
                ball_centered = cam._adjust_ball_coordinate_frame(ball)
                
            
            if cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        cam.stop()
        cv.destroyAllWindows()
    
if __name__ == "__main__":
    main()