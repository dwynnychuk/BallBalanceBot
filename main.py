import classCamera
import classControl
import classRobot
import classServo
from logger import get_logger
import time
import cv2 as cv
import math
import argparse

logger = get_logger(__name__)

def main(display: bool = False):
    # TODO servo init can be cleaned up
    servos: list[classServo.Servo] = classServo.init_servos(3)
    cam = classCamera.Camera()
    pid = classControl.PID()
    robot = classRobot.Robot()
    
    cam.start()
    
    # Initial Conditions
    setpoint = [0,0]
    desired_height = 0.13     # approximation
    CONTROL_HZ = 60
    CONTROL_DT = 1/CONTROL_HZ
    MAX_TILT_RAD = math.radians(13)
    TILT_THRES = 1e-6
    last_update = time.perf_counter()
    iteration = 0
    
    latency_log = []
    
    try:
        while True:
            now = time.perf_counter()
            if now - last_update < CONTROL_DT:
                time.sleep(0.001)
                continue
            
            last_update = now
            iteration += 1
            
            frame = cam.latest_frame
            ball = cam.get_ball_position()
            
            ball_age = cam.ball_age
            if ball_age is not None and ball_age > 0.1:  # 100ms stale
                logger.warning(f"Stale ball data: {ball_age*1000:.0f}ms old")
            
            if display and frame is not None:
                if ball_age is not None:
                    cv.putText(frame, f"Ball age: {ball_age*1000:.0f}ms", 
                             (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                cv.imshow("frame", cv.resize(frame, (640, 480)))
                
            if ball is not None:
                # Convert coordinates
                loop_start = time.perf_counter()
                
                ball_centered = cam._adjust_ball_coordinate_frame(ball)
                
                # Calculate tilt of platform
                pid_out = pid.compute_output(setpoint, ball_centered)
                
                tilt_x = max(min(pid_out[0], MAX_TILT_RAD), -MAX_TILT_RAD)
                tilt_y = max(min(pid_out[1], MAX_TILT_RAD), -MAX_TILT_RAD)
                
                tilt_mag = math.sqrt(tilt_x**2 + tilt_y**2)
                
                if tilt_mag > TILT_THRES:
                    nx = math.sin(tilt_x)
                    ny = math.sin(tilt_y)
                    nz = math.cos(tilt_mag)
                else:
                    nx, ny, nz = 0.0, 0.0, 1.0
                pid_norm = math.sqrt(nx**2 + ny**2 + nz**2)
                if pid_norm < TILT_THRES:         # div/ 0. on startup
                    continue
                
                nVec = [nx/pid_norm, ny/pid_norm, nz/pid_norm]
                
                # Inverse Kinematics
                thetas = robot.kinematics_inv(nVec, desired_height)
                
                # Rotate Servos
                for servo, theta in zip(servos, thetas):
                    servo.rotate_absolute(int(theta))
                    
                loop_time = (time.perf_counter() - loop_start)*1000
                latency_log.append(loop_time)
                
                if iteration % 100 == 0:
                    avg_latency = sum(latency_log[-100:]) / min(100, len(latency_log))
                    logger.info(f"Control loop: {avg_latency:.1f}ms avg, "
                               f"Ball age: {ball_age*1000:.0f}ms" if ball_age else "")
            
            if display and cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        cam.stop()
        if display:
            cv.destroyAllWindows()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--display",
        action = "store_true",
        help = "Enable OpenCV display window"
    )
    args = parser.parse_args()
    main(display=args.display)
