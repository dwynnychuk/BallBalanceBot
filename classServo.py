import platform
from logger import get_logger
from time import sleep

logger = get_logger(__name__)

def check_rpi() -> bool:
    logger.debug(f"Checking platform system: {platform.system()}")
    return platform.system() == "Linux"

is_pi: bool = check_rpi()
num_servos: int = 3

if is_pi:
    from adafruit_servokit import ServoKit
    logger.debug("Pi Detected: Adafruit Servo Kit imported")
else:
    ServoKit = None
    logger.debug("No Pi Detected: Adafruit Servo Kit not imported")

class Servo:
    def __init__(self, id: int, kit, homeAngle: int = 90):
        self.id = id
        self.kit = kit
        self.minAngle = 16
        self.maxAngle = 110
        self.homeAngle = homeAngle
        self.currentAngle = None
        logger.debug(f"ID: {self.id} Instantiated")
        
    def _run_position_offset(self, position: int) -> float:
        """Adjust coordinate frame such that 0degrees is a horizontal arm"""
        return 90 - position
        
    def rotate_absolute(self, position: int) -> None:        
        if position < self.minAngle:
            logger.warning(f"Demanded Angle of {position} on Servo {self.id} Less than minimum!")
            position = self.minAngle
            
        elif position > self.maxAngle:
            logger.warning(f"Demanded Angle of {position} on Servo {self.id} Greater than maximum!")
            position = self.maxAngle
            
        else:
            logger.debug(f"Moving Servo {self.id} to: {position}")
            
        if self.kit:
            self.kit.servo[self.id].angle = position
            self.currentAngle = position
    
    def reset(self) -> None:
        logger.debug(f"Homed Servo {self.id} to {self.homeAngle}")
        self.currentAngle = self.homeAngle
        if self.kit:
            self.kit.servo[self.id].angle = self.homeAngle


def home_all(servos: list[Servo]) -> None:
    for servo in servos:
        servo.reset()
    logger.debug("All servos homed")
    
def rotate_all(servos: list[Servo], angle: int) -> None:
    for servo in servos:
        servo.rotate_absolute(angle)
    
def _initialize_servo_range(servo_hat, num_servos: int, lower_limit=500, upper_limit=2500) -> None:
    """Open up servo range to unlock full range of motion"""
    for i in range(num_servos):
        servo_hat.servo[i].set_pulse_width_range(lower_limit, upper_limit)
    logger.debug(f"{num_servos} Servos pulse width limits modified to {lower_limit} -> {upper_limit}")

def init_servos(num_servos: int) -> list:
    try:
        if is_pi:
            servo_hat = ServoKit(channels = 16)
            _initialize_servo_range(servo_hat, num_servos)
            logger.debug("Pi Detected: 16 Channels Initialized")
        else:
            servo_hat = None
            logger.debug("No Pi Detected: No Servo Channels initialized")
            
        servos = [Servo(i, servo_hat) for i in range (num_servos)]
        home_all(servos)
        return servos
    except Exception as e:
        print(f"Error: initializing Servos {e}")    
        logger.error(f"Error initializing servos: {e}")
        
def forward_kinematics_sweep(
    servos: list[Servo],
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
        rotate_all(servos, angle)
        sleep(dwell)

    for angle in range(stop, start - step, -step):
        logger.info(f"FK TEST → rotate_all({angle})")
        rotate_all(servos, angle)
        sleep(dwell)

    logger.info("=== Forward Kinematics Sweep END ===")
        
def test_servos(servos: list[Servo]) -> None:
    rotate_all(servos, 90, True)    
    sleep(1)
    rotate_all(servos, 0, True)
    sleep(1)
    for i in range(10,100,10):
        rotate_all(servos, i, True)
        sleep(0.1)
    sleep(1)
    for i in range(90, -10, -1):
        rotate_all(servos, i, True)
        sleep(0.05)
    home_all(servos)
    servos[0].rotate_absolute(45, True)
    sleep(1)
    servos[0].rotate_absolute(0, True)
    servos[1].rotate_absolute(45, True)
    sleep(1)
    servos[0].rotate_absolute(0, True)
    servos[1].rotate_absolute(0, True)
    servos[2].rotate_absolute(45, True)
    sleep(1)
    home_all(servos)
    
if __name__ == "__main__":
    servos = init_servos(num_servos)
    forward_kinematics_sweep(servos)
