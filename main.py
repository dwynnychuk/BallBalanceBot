import classCamera
import classControl
import classRobot
import classServo
from logger import get_logger

logger = get_logger(__name__)

servos = classServo.init_servos(10)

def main():
    cam = classCamera.Camera()
    cam.show_camera()
    
if __name__ == "__main__":
    main()