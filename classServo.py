import platform

def check_rpi() -> bool:
    return platform.system() == "Linux"

is_pi = check_rpi()

if is_pi:
    from adafruit_servokit import ServoKit
    print("is pi")
else:
    ServoKit = None
    print("not a pi")

class Servo:
    def __init__(self, id, kit):
        self.id = id
        self.kit = kit
        self.minAngle = 0
        self.maxAngle = 180
        self.homeAngle = 90
        print(f"ID: {self.id}")
        
    def rotate(self, position):
        if position < self.minAngle:
            print(f"Demanded Angle of {position} on Servo {self.id} Less than minimum!")
        elif position > self.maxAngle:
            print(f"Demanded Angle of {position} on Servo {self.id} Greater than maximum!")
        else:
            print(f"Moving Servo {self.id} to: {position}")
            if self.kit:
                self.kit.servo[self.id].angle = position
    
    def reset(self):
        print(f"Homed Servo {self.id} to {self.homeAngle}")
        if self.kit:
            self.kit.servo[self.id].angle = self.homeAngle


def home_all(servos: list) -> None:
    for servo in servos:
        servo.reset()
    
def init_servos(num_servos) -> None:
    try:
        if is_pi:
            servo_hat = ServoKit(channels = 16)
        else:
            servo_hat = None
            
        servos = [Servo(i, servo_hat) for i in range (num_servos)]
        home_all(servos)
    except Exception as e:
        print(f"Error: initializing Servos {e}")    

if __name__ == "__main__":
    init_servos()
