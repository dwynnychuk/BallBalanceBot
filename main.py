import classCamera
import classControl
import classRobot
import classServo

# mock servo implementation
servo0 = classServo.Servo(0)
servo1 = classServo.Servo(1)
classServo.home_position([servo0, servo1])

def main():
    cam = classCamera.Camera()
    
if __name__ == "__main__":
    main()