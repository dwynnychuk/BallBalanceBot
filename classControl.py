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

@dataclass
class PIDState:
    """State variables for the PID controller of a given axis"""
    error: float = 0.0
    velocity: float = 0.0
    integral: float = 0.0
    prev_measurement: Optional[float] = None

class PID:
    """Dual axis PID controller for ball balancing robot
    """
    def __init__(
        self,
        gains: Optional[PIDGains] = None,
        deadband: float = 15.0,
        max_integral: float = 0.005
    ):
        self.gains = gains if gains is not None else PIDGains()
        self.deadband = deadband
        self.max_integral = max_integral
        
        self.state_x = PIDState()
        self.state_y = PIDState()
        
        self.prev_time: Optional[float] = None
        
        logger.debug("PID initialized: "
                     f"kp: {self.gains.kp}, "
                     f"ki: {self.gains.ki}, "
                     f"kd: {self.gains.kd}, "
                     f"alpha: {self.gains.alpha}, "
                     f"deadband px: {self.deadband}")
        
    def compute_output(
        self, 
        setpoint: Tuple[float, float], 
        measurement: Tuple[float, float]
        ) -> Tuple[float, float]:
        """
        Compute desired pose of platform
        
        Args:
        
        Returns:
            Tuple of (x_out, y_out) PID controller for x and y axis
        """
        current_time = time.perf_counter()
        
        # Initialize first call or after reset
        if self.prev_time is None:
            self._initialize_state(measurement, current_time)
            return (0.0, 0.0)

        dt = current_time - self.prev_time
        # Check dt
        if dt <= 0:
            logger.warning(f"Error: Invalid dt: {dt}, skipping control update")
            return self._get_current_output()
        
        # Log input
        logger.debug(f"PID INPUT: x: {measurement[0]:.2f}, y: {measurement[1]:.2f}")
        
        out_x = self._compute_axis_output(setpoint[0], measurement[0], self.state_x, dt, 'X')
        out_y = self._compute_axis_output(setpoint[1], measurement[1], self.state_y, dt, 'Y')
        
        self.prev_time = current_time
        
        return [-out_x, out_y]
    
    def _compute_axis_output(self,
                             setpoint: float,
                             measurement: float,
                             state: PIDState, 
                             dt: float,
                             axis_name: str
                             ) -> float:
        """_summary_

        Args:
            setpoint (float): Desired position
            measurement (float): Current position
            state (PIDState): PID state for this axis
            dt (float): Time between control iterations
            axis_name (str): 'X' or 'Y' axis

        Returns:
            float: Control output
        """
        error: float = setpoint - measurement
        
        if abs(error) < self.deadband:
            error = 0.0
        
        # Calculate integral term with windup clamping
        state.integral += error * dt
        state.integral = self._clamp(state.integral, -self.max_integral, self.max_integral)
        
        # Calculate derivative value with filtering
        if state.prev_measurement is not None:
            velocity_raw = (measurement - state.prev_measurement) / dt
            state.velocity = self.gains.alpha * state.velocity + (1 - self.gains.alpha) * velocity_raw
        
        # Compute PID
        out_p = self.gains.kp * error
        out_i = self.gains.ki * state.integral
        out_d = -self.gains.kd * state.velocity
        
        output = out_p + out_i + out_d
        
        # Update state
        state.error = error
        state.prev_measurement = measurement
        
        logger.debug(f"PID OUTPUT {axis_name} AXIS: P: {out_p}, I: {out_i}, D: {out_d} -> Total: {output}")
        
        return output
    
    def _initialize_state(self, measurement: Tuple[float, float], current_time: float) -> None:
        """Initialize state of PID controller

        Args:
            measurement (Tuple[float, float]): Measured position
            current_time (float): Time of initialization
        """
        self.state_x.prev_measurement = measurement[0]
        self.state_y.prev_measurement = measurement[1]
        self.prev_time = current_time
        logger.debug("PID State initialized")
    
    def _get_current_output(self) -> Tuple[float, float]:
        """Calculate and output current state of PID controller

        Returns:
            Tuple[float, float]: PID output based on current state
        """
        out_x = (self.gains.kp * self.state_x.error +
                 self.gains.ki * self.state_x.integral - 
                 self.gains.kd * self.state_x.velocity)
        out_y = (self.gains.kp * self.state_y.error +
                 self.gains.ki * self.state_y.integral - 
                 self.gains.kd * self.state_y.velocity)
        
        return (-out_x, out_y)
    
    def reset(self) -> None:
        """Reset PID controller for fresh slate"""
        self.state_x = PIDState()
        self.state_y = PIDState()
        self.prev_time = None
        logger.debug("PID Controller Reset")
    
    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        """Clamp value between minimum and maximum values"""
        return max(min_value, min(value, max_value))
