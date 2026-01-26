from logger import get_logger
import time

logger = get_logger(__name__)

class PID:
    def __init__(self):
        self.kp = 0.0008
        self.ki = 0.0
        self.kd = 0.000015
        self.t0 = None
        self.tn1 = None
        self.dt = None
        self.prev_error_x = 0
        self.prev_error_y = 0
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
            return [self.out_x, self.out_y]

        error_x = setpoint[0] - measurement[0]
        error_y = setpoint[1] - measurement[1]
        dt = self.t0 - self.tn1
        if dt <= 0:
            return [self.out_x, self.out_y]
        
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        
        derivative_x = (error_x - self.prev_error_x)/dt
        derivative_y = (error_y - self.prev_error_y)/dt
        
        self.out_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        self.out_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.tn1 = self.t0
        
        #logger.debug(f"PID Output -> x: {self.out_x}, y: {self.out_y}")
        
        # Return -y as arm 2 - 3 math is reversed in Y
        #TODO fix this in IK
        return [-self.out_x, self.out_y]
