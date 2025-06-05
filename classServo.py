class Servo:
    def __init__(self, id):
        self.id = id
        print(f"ID: {self.id}")
        
    def rotate_servo(self, position):
        print(position)
    

servo = Servo(1)

servo.rotate_servo(100)