from adafruit_servokit import ServoKit

servoHat = ServoKit(channels=16)

class Servo:
    def __init__(self, id):
        self.id = id
        self.minAngle = 0
        self.maxAngle = 180
        self.homeAngle = 90
        print(f"ID: {self.id}")
        
    def _rotate_servo(self, position):
        if position < self.minAngle:
            print(f"Demanded Angle of {position} on Servo {self.id} Less than minimum!")
        elif position > self.maxAngle:
            print(f"Demanded Angle of {position} on Servo {self.id} Greater than maximum!")
        else:
            print(f"Moving Servo {self.id} to: {position}")
            servoHat.servo[self.id].angle = position
    
    def _reset_servo(self):
        servoHat.servo[self.id].angle = self.homeAngle
        print(f"Homed Servo {self.id} to {self.homeAngle}")


def home_position(servos: list) -> None:
    for servo in servos:
        servo._reset_servo()
    
