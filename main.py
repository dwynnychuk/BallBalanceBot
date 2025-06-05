import classCamera
import classControl
import classRobot
import classServo

# mock servo implementation
servos = classServo.init_servos(10)

def main():
    cam = classCamera.Camera()
    cam.show_camera()
    
if __name__ == "__main__":
    main()