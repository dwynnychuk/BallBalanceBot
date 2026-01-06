import numpy as np
import matplotlib.pyplot as plt
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

# --- Manipulator geometry ---
L1 = 0.080   # first link length
L2 = 0.105   # second link length
base_positions = np.array([
    [0.046, 0, 0],       # base of leg 1
    [-0.023, 0.0398, 0], # base of leg 2
    [-0.023, -0.0398, 0] # base of leg 3
])

# Servo limits
servo_min = 16
servo_max = 110

# --- Forward kinematics for 3-RRS ---
def fk_3rrs(angles):
    """Angles in degrees. Returns (x,y,z) or None if unreachable."""
    angles = np.radians(angles)
    elbows = np.array([
        base_positions[i] + L1 * np.array([np.cos(angles[i]), 0, np.sin(angles[i])])
        for i in range(3)
    ])
    P1, P2, P3 = elbows
    r1 = r2 = r3 = L2

    ex = (P2 - P1) / np.linalg.norm(P2 - P1)
    i = np.dot(ex, P3 - P1)
    ey = P3 - P1 - i * ex
    ey /= np.linalg.norm(ey)
    ez = np.cross(ex, ey)
    d = np.linalg.norm(P2 - P1)
    j = np.dot(ey, P3 - P1)

    x = (r1**2 - r2**2 + d**2) / (2*d)
    y = (r1**2 - r3**2 + i**2 + j**2)/(2*j) - (i/j)*x
    z2 = r1**2 - x**2 - y**2
    if z2 < 0:
        return None
    z = np.sqrt(z2)
    pos = P1 + x*ex + y*ey + z*ez
    return pos

# --- Sweep IK angles and map to servo angles ---
ik_angles = np.arange(0, 91, 5)  # 0° vertical, 90° bent
servo_angles_mapped = servo_max - ik_angles  # map IK -> servo
servo_angles_mapped = np.clip(servo_angles_mapped, servo_min, servo_max)

reachable_z_ik = []
reachable_z_servo = []

unreachable_ik = []
unreachable_servo = []

for ik_angle, servo_angle in zip(ik_angles, servo_angles_mapped):
    # FK using mapped servo angle
    pos_servo = fk_3rrs([servo_angle]*3)
    if pos_servo is not None:
        reachable_z_servo.append(pos_servo[2])
    else:
        reachable_z_servo.append(np.nan)
        unreachable_servo.append(servo_angle)

    # FK using IK as if it were servo (to compare)
    pos_ik = fk_3rrs([ik_angle]*3)
    if pos_ik is not None:
        reachable_z_ik.append(pos_ik[2])
    else:
        reachable_z_ik.append(np.nan)
        unreachable_ik.append(ik_angle)

# --- Plot ---
plt.figure(figsize=(9,5))
plt.plot(ik_angles, reachable_z_servo, marker='o', label='FK z-height (servo mapped from IK)')
plt.plot(ik_angles, reachable_z_ik, marker='x', label='FK z-height (direct IK as angle)')
plt.xlabel('IK Angle (deg, 0° vertical)')
plt.ylabel('End-effector z-height')
plt.title('3-RRS FK z-height vs IK Angle')
plt.grid(True)
plt.legend()
plt.show()

logging.info(f"Unreachable servo angles: {unreachable_servo}")
logging.info(f"Unreachable IK angles: {unreachable_ik}")
