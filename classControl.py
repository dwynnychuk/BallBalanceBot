from logger import get_logger
from dataclasses import dataclass
from typing import Tuple, Optional
import time

logger = get_logger(__name__)

@dataclass 
class PIDGains:
    """PID gains"""
    kp: float = 0.00006
    ki: float = 0.0000002
    kd: float = 0.0001
    alpha: float = 0.7

class PID:
    def __init__(
        self,
        gains: Optional[PIDGains] = None,
        deadband: float = 15.0,
        max_integral: float = 0.005
    ):
        self.gains = gains or PIDGains()
        self.deadband = deadband
        self.max_integral = max_integral
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
        
        if abs(error_x) < self.deadband:
            error_x = 0
        
        if abs(error_y) < self.deadband:
            error_y = 0
        
        dt = self.t0 - self.tn1
        if dt <= 0:
            return [self.out_x, self.out_y]
        
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        
        self.integral_x = max(-self.max_integral, min(self.max_integral, self.integral_x))
        self.integral_y = max(-self.max_integral, min(self.max_integral, self.integral_y))
        
        vx_raw = (measurement[0] - self.prev_measurement_x)/dt
        vy_raw = (measurement[1] - self.prev_measurement_y)/dt
        
        self.vel_x = self.gains.alpha * self.vel_x + (1 - self.gains.alpha) * vx_raw
        self.vel_y = self.gains.alpha * self.vel_y + (1 - self.gains.alpha) * vy_raw
        
        out_px = self.gains.kp * error_x
        out_ix = self.gains.ki * self.integral_x
        out_dx = - self.gains.kd * self.vel_x
        
        out_py = self.gains.kp * error_y
        out_iy = self.gains.ki * self.integral_y
        out_dy = - self.gains.kd * self.vel_y
        
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
    
    def _compute_axis_output():
        pass
    
    def reset(self):
        pass
    
    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        """Clamp value between minimum and maximum values"""
        return max(min_value, min(value, max_value))
