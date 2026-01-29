import platform
from logger import get_logger
from time import sleep

logger = get_logger(__name__)

def check_rpi() -> bool:
    """Check system hardware to determine if running on raspberry pi

    Returns:
        bool: True if running linux ie. rPi
    """
    logger.debug(f"Checking platform system: {platform.system()}")
    return platform.system() == "Linux"

is_pi: bool = check_rpi()

if is_pi:
    from adafruit_servokit import ServoKit
    logger.debug("Pi Detected: Adafruit Servo Kit imported")
else:
    ServoKit = None
    logger.debug("No Pi Detected: Adafruit Servo Kit not imported")

class ServoHardwareError(RuntimeError):
    pass

class Servo:
    """Main class for individual servo control
    """
    ### --- CLASS CONSTANTS --- ###
    MIN_ANGLE: int = 16
    MAX_ANGLE: int = 110
    HOME_ANGLE_DEFAULT: int = 90
    COORDINATE_OFFSET: int = 90
    SERVO_LOWER_PWM_LIMIT: int = 500
    SERVO_UPPER_PWM_LIMIT: int = 2500
    
    def __init__(self, id: int, kit, home_angle = None):
        self.id = id
        self.kit = kit
        self.minAngle = self.MIN_ANGLE
        self.maxAngle = self.MAX_ANGLE
        self.home_angle = home_angle if home_angle is not None else self.HOME_ANGLE_DEFAULT
        self.currentAngle = None
        logger.debug(f"ID: {self.id} Instantiated")
        
    def _run_position_offset(self, position: int) -> int:
        """Adjust coordinate frame such that 0degrees is a horizontal arm"""
        return 90 - position
        
    def rotate_absolute(self, position: int) -> int:        
        """Rotates individual servo within MIN_ANGLE and MAX_ANGLE

        Args:
            position (int): angular position of servo before any offset
            
        Returns: 
            Angle of servo to know if demanded was achieved.
        """
        servo_angle = self._run_position_offset(position)
        
        if servo_angle < self.minAngle:
            logger.warning(f"IK Angle: {position} -> Servo Angle: {servo_angle} on Servo {self.id} Less than minimum!")
            servo_angle = self.minAngle
            
        elif servo_angle > self.maxAngle:
            logger.warning(f"IK Angle: {position} -> Servo Angle: {servo_angle} on Servo {self.id} Greater than maximum!")
            servo_angle = self.maxAngle
            
        else:
            #logger.debug(f"Moving Servo {self.id} to: IK Angle: {position} -> Servo Angle {servo_angle}")
            pass
            
        if self.kit:
            try:
                self.kit.servo[self.id].angle = servo_angle
                self.currentAngle = servo_angle
            except Exception as e:
                raise ServoHardwareError(f"Servo: {self.id} failed to move") from e
        
        return servo_angle
    
    def reset(self) -> None:
        logger.debug(f"Homed Servo {self.id} to {self.home_angle}")
        self.currentAngle = self.home_angle
        if self.kit:
            self.kit.servo[self.id].angle = self.home_angle

    @staticmethod
    def _initialize_servo_range(
        servo_hat, num_servos: int, 
        lower_limit=None, 
        upper_limit=None
        ) -> None:
        """Open up servo range to unlock full range of motion"""
        lower_limit = lower_limit if lower_limit is not None else Servo.SERVO_LOWER_PWM_LIMIT
        upper_limit = upper_limit if upper_limit is not None else Servo.SERVO_UPPER_PWM_LIMIT
        try:
            for i in range(num_servos):
                servo_hat.servo[i].set_pulse_width_range(lower_limit, upper_limit)
            logger.debug(f"{num_servos} Servos pulse width limits modified to {lower_limit} -> {upper_limit}")
        except Exception as e:
            raise ServoHardwareError("Failed to initialize ServoKit") from e

class ServoGroup:
    """Class to abstract group of servos and apply bulk methods
    """
    def __init__(self, servos: list[Servo]):
        self.servos = servos
        
    def home_all(self) -> None:
        """Rotate all servos to home position."""
        for servo in self.servos:
            servo.reset()
        logger.debug("All servos homed")
    
    def rotate_all(self, angle: int) -> None:
        """Rotate all servos to specified angle."""
        for servo in self.servos:
            servo.rotate_absolute(angle)
        logger.debug(f"All servos set to angle: {angle}")
        
    def get_servo(self, index: int) -> Servo:
        """Get individual servo control."""
        return self.servos[index]

def init_servos(num_servos: int) -> ServoGroup:
    """Initialize and home all servo objects

    Args:
        num_servos (int): Number of servos in design.

    Returns:
        ServoGroup: Object of a list containing all Servo instance objects.
    """
    try:
        if is_pi:
            servo_hat = ServoKit(channels = 16)
            Servo._initialize_servo_range(servo_hat, num_servos)
            logger.debug("Pi Detected: 16 Channels Initialized")
        else:
            servo_hat = None
            logger.debug("No Pi Detected: No Servo Channels initialized")
            
        servos = [Servo(i, servo_hat) for i in range (num_servos)]
        
        servo_group = ServoGroup(servos)
        servo_group.home_all()
        
        return servo_group
    
    except Exception as e:
        logger.critical(f"Error initializing servos: {e}")
        raise
        
def forward_kinematics_sweep(
    servo_group: ServoGroup,
    start: int = -10,
    stop: int = 100,
    step: int = 2,
    dwell: float = 0.5
) -> None:
    """
    Sweep all servos together to visually observe platform motion.
    This acts as a forward-kinematics diagnostic.
    """

    logger.info("=== Forward Kinematics Sweep START ===")

    for angle in range(start, stop + step, step):
        logger.info(f"FK TEST → rotate_all({angle})")
        servo_group.rotate_all(angle)
        sleep(dwell)

    for angle in range(stop, start - step, -step):
        logger.info(f"FK TEST → rotate_all({angle})")
        servo_group.rotate_all(angle)
        sleep(dwell)

    logger.info("=== Forward Kinematics Sweep END ===")
    
if __name__ == "__main__":
    num_servos: int = 3
    servo_group = init_servos(num_servos)
    forward_kinematics_sweep(servo_group)
