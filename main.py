import classCamera
import classControl
import classRobot
import classServo
from logger import get_logger
import time
import cv2 as cv
import math

logger = get_logger(__name__)

def main():
    # TODO servo init can be cleaned up
    servos: list[classServo.Servo] = classServo.init_servos(3)
    cam = classCamera.Camera()
    pid = classControl.PID()
    robot = classRobot.Robot()
    
    cam.start()
    
    # Initial Conditions
    setpoint = [0,0]
    desired_height = 0.05     # approximation
    
    try:
        while True:
            frame = cam.latest_frame
            ball = cam.get_ball_position()
            
            if frame is not None:
                cv.imshow("frame", cv.resize(frame, (640,480)))
                
            if ball is not None:
                # Convert coordinates
                ball_centered = cam._adjust_ball_coordinate_frame(ball)
                
                # Calculate tilt of platform
                pid_out = pid.compute_output(setpoint, ball_centered)
                nx, ny = pid_out
                nz = 1
                pid_norm = math.sqrt(nx**2 + ny**2 + nz**2)
                if pid_norm < 1e-6:         # div/ 0. on startup
                    continue
                
                nVec = [nx/pid_norm, ny/pid_norm, nz/pid_norm]
                
                # Inverse Kinematics
                thetas = robot.kinematics_inv(nVec, desired_height)
                
                # Rotate Servos
                for servo, theta in zip(servos, thetas):
                    servo.rotate_absolute(int(theta))
            
            if cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        cam.stop()
        cv.destroyAllWindows()
    
if __name__ == "__main__":
    main()