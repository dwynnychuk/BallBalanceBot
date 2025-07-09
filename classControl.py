from logger import get_logger
import time

logger = get_logger(__name__)

class PID:
    def __init__(self):
        self.kp = 1
        self.ki = 0.0
        self.kd = 0.0
        self.t0 = None
        self.tn1 = None
        self.dt = None
        self.error_x = 0
        self.error_y = 0
        self.integral_x = 0
        self.integral_y = 0
        self.out_x = 0
        self.out_y = 0
        
        
    def compute_output(self, setpoint, measurement):
        # computer P I D
        # compute output
        # relate to coordinate system
        # update old values
        self.t0 = time.perf_counter()
        if self.tn1 is None:
            self.tn1 = self.t0
            return [self.out_x, self.out_y]

        self.error_x = measurement[0] - setpoint[0]
        self.error_y = measurement[1] - setpoint[1]
        dt = self.t0 - self.tn1
        
        self.integral_x += self.error_x
        self.integral_y += self.error_y
        
        derivative_x = self.error_x/dt
        derivative_y = self.error_y/dt
        
        self.out_x = self.kp * self.error_x + self.ki * self.integral_x + self.kd * derivative_x
        self.out_y = self.kp * self.error_y + self.ki * self.integral_y + self.kd * derivative_y
        
        
        
     