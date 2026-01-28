from logger import get_logger
import time

logger = get_logger(__name__)

class PID:
    def __init__(self):
        self.kp = 0.00013
        self.ki = 0.00018
        self.kd = 0.00016
        self.alpha = 0.7
        self.t0 = None
        self.tn1 = None
        self.dt = None
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.prev_measurement_x = None
        self.prev_measurement_y = None
        self.vel_x = 0
        self.vel_y = 0
        self.integral_x = 0
        self.integral_y = 0
        self.out_x = 0
        self.out_y = 0
        logger.debug("PID class initialized")
        
        
    def compute_output(self, setpoint, measurement):
        """
        Compute desired pose of platform
        INPUTS:
        OUTPUTS: 
        TODO clamping
        """
        self.t0 = time.perf_counter()
        if self.tn1 is None:
            self.tn1 = self.t0
            self.prev_measurement_x = measurement[0]
            self.prev_measurement_y = measurement[1]
            return [self.out_x, self.out_y]

        logger.debug(f"PID INPUT: x: {measurement[0]}, y: {measurement[1]}")
        error_x = setpoint[0] - measurement[0]
        error_y = setpoint[1] - measurement[1]
        
        dt = self.t0 - self.tn1
        if dt <= 0:
            return [self.out_x, self.out_y]
        
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        
        vx_raw = (measurement[0] - self.prev_measurement_x)/dt
        vy_raw = (measurement[1] - self.prev_measurement_y)/dt
        
        self.vel_x = self.alpha * self.vel_x + (1 - self.alpha) * vx_raw
        self.vel_y = self.alpha * self.vel_y + (1 - self.alpha) * vy_raw
        
        out_px = self.kp * error_x
        out_ix = self.ki * self.integral_x
        out_dx = - self.kd * self.vel_x
        
        out_py = self.kp * error_y
        out_iy = self.ki * self.integral_y
        out_dy = - self.kd * self.vel_y
        
        self.out_x = out_px + out_ix + out_dx
        self.out_y = out_py + out_iy + out_dy
        
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        
        self.prev_measurement_x = measurement[0]
        self.prev_measurement_y = measurement[1]
        self.tn1 = self.t0
        
        logger.debug(f"PID Output -> x-> P: {out_px}, I: {out_ix}, D: {out_dx}, y-> P: {out_py}, I: {out_iy}, D: {out_dy}")
        
        # Return -y as arm 2 - 3 math is reversed in Y
        #TODO fix this in IK
        return [-self.out_x, self.out_y]
