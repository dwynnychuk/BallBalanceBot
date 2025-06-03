class PID:
    def __init__(self, gains):
        self.kp = gains[0]
        self.ki = gains[1]
        self.kd = gains[2]
        
    def compute_output(self, setpoint, measurement):
        # computer P I D
        # compute output
        # relate to coordinate system
        # update old values
        pass
    