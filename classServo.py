import platform
from logger import get_logger

logger = get_logger(__name__)

def check_rpi() -> bool:
    logger.debug(f"Checking platform system: {platform.system()}")
    return platform.system() == "Linux"

is_pi = check_rpi()

if is_pi:
    from adafruit_servokit import ServoKit
    logger.debug("Pi Detected: Adafruit Servo Kit imported")
else:
    ServoKit = None
    logger.debug("No Pi Detected: Adafruit Servo Kit not imported")

class Servo:
    def __init__(self, id, kit, homeAngle = 90):
        self.id = id
        self.kit = kit
        self.minAngle = 0
        self.maxAngle = 90
        self.homeAngle = homeAngle
        self.currentAngle = None
        logger.debug(f"ID: {self.id} Instantiated")
        
    def rotate_absolute(self, position):
        if position < self.minAngle:
            logger.warning(f"Demanded Angle of {position} on Servo {self.id} Less than minimum!")
        elif position > self.maxAngle:
            logger.warning(f"Demanded Angle of {position} on Servo {self.id} Greater than maximum!")
        else:
            logger.debug(f"Moving Servo {self.id} to: {position}")
            if self.kit:
                self.kit.servo[self.id].angle = position
    
    def reset(self):
        logger.debug(f"Homed Servo {self.id} to {self.homeAngle}")
        self.currentAngle = self.homeAngle
        if self.kit:
            self.kit.servo[self.id].angle = self.homeAngle


def home_all(servos: list) -> None:
    for servo in servos:
        servo.reset()
    logger.debug("All servos homed")
    
def init_servos(num_servos=3) -> None:
    try:
        if is_pi:
            servo_hat = ServoKit(channels = 16)
            logger.debug("Pi Detected: 16 Channels Initialized")
        else:
            servo_hat = None
            logger.debug("No Pi Detected: No Servo Channels initialized")
            
        servos = [Servo(i, servo_hat) for i in range (num_servos)]
        home_all(servos)
    except Exception as e:
        print(f"Error: initializing Servos {e}")    
        logger.error(f"Error initializing servos: {e}")

if __name__ == "__main__":
    init_servos()
