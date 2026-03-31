# BallBalanceBot
## Summary
A 3-DOF robotic project that balances a ball in the center of a platform. 

<img width="694" height="665" alt="CAD_ISO" src="https://github.com/user-attachments/assets/26ecefea-4fcb-4339-8aa5-2e4b908fbe63" />

## Structure
    .
    ├── main.py
    ├── cameraCalibration.py
    ├── classCamera.py
    ├── classControl.py
    ├── classRobot.py
    ├── classServo.py
    └── logger.py
    
## Usage
Without camera display 
`python3 main.py`

With camera display
`python3 main.py --display`

Camera Calibration (Only required once per system)
`python3 cameraCalibration.py`

## RequirementsBallBalanceBot
- numpy
- opencv-python
- matplotlib
- adafruit-circuitpython-servokit (Raspberry Pi Only)
- picamera2 (Raspberry Pi Only)

## CAD
- [3x] `arm_1_drive.step`
- [3x] `arm_1_driven.step`
- [6x] `arm_2.step`
- [3x] `bottom_clamp.step`
- [1x] `pi_base.step`
- [1x] `pi_middle.step`
- [3x] `pi_spacer.step`
- [4x] `pi_standoff.step`
- [1x] `pi_top.step`
- [1x] `servo_holder.step`
- [3x] `top_clamp.step`
- [1x OPTIONAL] `drill_guide.step`

## Additional Hardware
- [1x] [Raspberry Pi 3B+](https://www.amazon.com/Raspberry-Pi-Model-Board-Plus/dp/B0BNJPL4MW/)
- [1x] [Adafruit Servo Hat](https://www.amazon.com/Adafruit-16-Channel-12-bit-Servo-Shield/dp/B00I4WMOGE/)
- [3x] [Servos](https://www.amazon.com/Deegoo-FPV-Servo-MG995-Metal-Gear/dp/B07NQJ1VZ2/)
- [1x] [6" Acrylic Plate](https://www.amazon.com/Pieces-Plexiglass-Diameter-Acrylic-Backdrop/dp/B0C7TMYW9J/)
- [1x] [Arducam Module](https://www.amazon.com/dp/B0BZR6XL3Y?ref=ppx_yo2ov_dt_b_fed_asin_title)
- [3x] [Ball Joints](https://www.amazon.com/HobbyPark-Universal-Plastic-Turnbuckle-Airplane/dp/B07Z1MDC7H/) 
- [9x] [Ball Bearings](https://www.amazon.com/uxcell-Groove-Bearing-2080093-Bearings/dp/B07FVYPMPX/)
