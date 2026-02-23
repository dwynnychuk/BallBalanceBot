import classCamera
import classControl
import classRobot
import classServo
from classServo import ServoGroup
from logger import get_logger
import time
import cv2 as cv
import math
import argparse

logger = get_logger(__name__)

def main(display: bool = False):
    NUM_SERVOS: int = 3    
    CONTROL_HZ: int = 60
    CONTROL_DT: float = 1/CONTROL_HZ
    MAX_TILT_RAD: float = math.radians(13)
    TILT_THRES: float = 1e-6
    SETPOINT: list = [0,0]
    DESIRED_HEIGHT: float = 0.13
    BALL_TIMEOUT: float = 1.0
    
    servo_group: ServoGroup = classServo.init_servos(NUM_SERVOS)
    pid = classControl.PID()
    robot = classRobot.Robot()
    
    # Initial Conditions
    last_update: float = time.perf_counter()
    iteration: int = 0
    latency_log: list = []
    platform_reset: bool = False
    
    with classCamera.Camera() as cam:
        logger.info("Camera class started")
        
        try:
            while True:
                now = time.perf_counter()
                if now - last_update < CONTROL_DT:
                    time.sleep(0.001)
                    continue
                
                last_update = now
                iteration += 1
                
                frame_data = cam.get_latest_frame()
                if frame_data is not None:
                    frame, timestamp = frame_data
                    
                ball_age = cam.ball_age
                if ball_age is None or ball_age > BALL_TIMEOUT:
                    if not platform_reset:
                        logger.warning(f"Ball lost for {BALL_TIMEOUT}s, Resetting Platform")
                        servo_group.home_all()
                        pid.reset()
                        platform_reset = True
                    continue
                
                if platform_reset:
                    logger.info("Ball reacquired. Resuming control.")
                    pid.reset()
                    platform_reset = False
                    
                if cam.enable_visualization and frame is not None:
                    if ball_age is not None:
                        cv.putText(
                            frame, 
                            f"Ball age: {ball_age*1000:.0f}ms", 
                            (10, 90), 
                            cv.FONT_HERSHEY_SIMPLEX, 
                            0.6, (0,255,255), 2
                            )
                    cv.imshow("frame", cv.resize(frame, (640, 480)))                    

                ball_pos = cam.get_ball_position_robot_frame()                    
                if ball_pos is not None:
                    
                    loop_start = time.perf_counter()
                                        
                    # Calculate tilt of platform
                    pid_out = pid.compute_output(SETPOINT, ball_pos)
                    
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
                    
                    normal_vector = [nx/pid_norm, ny/pid_norm, nz/pid_norm]
                    
                    # Inverse Kinematics
                    thetas = robot.kinematics_inv(normal_vector, DESIRED_HEIGHT)
                    
                    # Rotate Servos
                    servo_group.apply_angles(thetas)
                        
                    loop_time = (time.perf_counter() - loop_start)*1000
                    latency_log.append(loop_time)
                    
                    if iteration % 100 == 0:
                        avg_latency = sum(latency_log[-100:]) / min(100, len(latency_log))
                        logger.info(f"Control loop: {avg_latency:.1f}ms avg, "
                                f"Ball age: {ball_age*1000:.0f}ms" if ball_age else "")
                
                if cam.enable_visualization and cv.waitKey(1) & 0xFF == 27:
                    break
        finally:
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
