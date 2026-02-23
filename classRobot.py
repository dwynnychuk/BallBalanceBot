from logger import get_logger
from math import sqrt, degrees, atan2
from dataclasses import dataclass
from typing import List
import classServo
import time

logger = get_logger(__name__)

@dataclass
class LinkLengths:
    """
    Robot link lengths in meters.
    
    L0: Origin to Arm 1 Base
    L1: Arm 1 (Bottom) Length
    L2: Arm 2 (Top) Length
    L3: Plate radius
    """
    L0: float = 0.105
    L1: float = 0.08
    L2: float = 0.046
    L3: float = 0.0935
    
class Robot:
    # CONSTANTS
    _SQRT3: float = sqrt(3)
    _ANGLE_OFFSET: float = 90.0
    
    def __init__(self, 
                 links: LinkLengths = None):
        
        self.links = links if links is not None else LinkLengths()
        
        logger.debug("Robot class initialized")
    
    def kinematics_inv(self, 
                       normal_vector: list[float], 
                       height: float
                       ) -> list[float]:
        """
        Compute servo angles for all three arms based on desired plate orientation

        Args:
            normal_vector (list[float]): Platform normal vector [nx, ny, nz]
            height (float): Desired platform height [m]

        Returns:
            list[float]: required servo angles [theta1, theta2, theta3]
        """
        L = self.links
        
        # TODO Validate inputs
        
        reference_height = self._compute_reference_height(height)
        
        theta_1 = self._compute_arm_angle_1(normal_vector, height, reference_height)
        theta_2 = self._compute_arm_angle_2(normal_vector, height, reference_height)
        theta_3 = self._compute_arm_angle_3(normal_vector, height, reference_height)

        thetas = [theta_1, theta_2, theta_3]
        logger.debug(f"IK Solution: {[f'{t:.2f}' for t in thetas]}")
        
        return thetas
    
    def _compute_reference_height(self, 
                                  height: float
                                  ) -> float:
        """
        Compute reference Z position for elbow configuration check.
        
        This is the Z position at the reference (level) platform pose,
        used to determine elbow-up vs elbow-down configuration.

        Args:
            height (float): Platform height in meters

        Returns:
            float: Reference reference_height value
        """
        L = self.links
        h = height
        
        A = (L.L1 + L.L2)/h
        B = (L.L0**2 + h**2 - L.L3**2 - (L.L1 + L.L2)**2)/(2*h)
        C = A**2 + 1
        D = 2*(A*B - (L.L1 + L.L2))
        E = B**2 + (L.L1 + L.L2)**2 - L.L0**2
        
        # TODO value check here
        PX = (-D + sqrt(D**2 - 4*C*E))/(2*C)
        reference_height = sqrt(L.L0**2 - (PX - (L.L1 + L.L2))**2)
        
        return reference_height
    
    def _compute_arm_angle_1(self, 
                             n: List[float], 
                             h: float, 
                             reference_height: float
                             ) -> float:
        L = self.links
        
        denom = sqrt(n[0]**2 + n[2]**2)
        
        # TODO value check on denominator
        
        # Ball Joint
        bj_x = (L.L3*n[2])/denom
        bj_y = 0
        bj_z = h - (n[0]*L.L3)/denom
        
        A = (L.L2 - bj_x)/bj_z
        B = (bj_x**2 + bj_y**2 + bj_z**2 + L.L1**2 - L.L0**2 - L.L2**2) / (2*bj_z)
        C = A**2 + 1
        D = 2*(A*B - L.L2)
        E = B**2 + L.L2**2 - L.L1**2
        
        pj_x = (-D + sqrt(D**2 - (4*C*E)))/(2*C)
        pj_z = sqrt(L.L1**2 - (pj_x - L.L2)**2)
        
        # check elbow up vs elbow down
        if bj_z < reference_height:
            pj_z = -pj_z
        
        theta_raw = degrees(atan2(pj_x - L.L2, pj_z))
        return self._ANGLE_OFFSET - theta_raw
        
    def _compute_arm_angle_2(self,
                             n: List[float],
                             h: float,
                             reference_height: float
                             ) -> float:
        L = self.links
        S3 = self._SQRT3
        
        denom = sqrt(4*n[2]**2 + n[0]**2 + 3*n[1]**2 - 2*S3*n[0]*n[1])
        #TODO Check denominator value
        
        # Ball Joint
        bj_x = -(L.L3 * n[2]) / denom
        bj_y = (S3 * L.L3 * n[2]) / denom
        bj_z = h + ((n[0] - S3 * n[1]) * L.L3) / denom
        
        A = (S3 * bj_y - 2 * L.L2 - bj_x) / bj_z
        B = (bj_x**2 + bj_y**2 + bj_z**2 + L.L1**2 - L.L0**2 - L.L2**2) / (2 * bj_z)
        C = A**2 + 4
        D = 2*(A * B + 2 * L.L2)
        E = B**2 + L.L2**2 - L.L1**2
        
        pj_x = (-D - sqrt(D**2 - 4 * C * E)) / (2 * C)
        pj_y = -S3 * pj_x
        pj_z = sqrt(L.L1**2 - 4 * pj_x**2 - 4 * L.L2 * pj_x - L.L2**2)
        
        # check elbow up vs elbow down
        if bj_z < reference_height:
            pj_z = -pj_z
            
        # adjust for physical build (0 -> vertical)
        radial = sqrt(pj_x**2 + pj_y**2)
        theta_raw = degrees(atan2(radial - L.L2, pj_z))
        return self._ANGLE_OFFSET - theta_raw
    
    def _compute_arm_angle_3(self,
                             n: List[float],
                             h: float,
                             reference_height: float
                             ) -> float:
        L = self.links
        S3 = self._SQRT3
        
        # FIX FROM HERE
        denom = sqrt(4 * n[2]**2 + n[0]**2 + 2 * S3 * n[0] * n[1] + 3 * n[1]**2)
        # TODO Check denominator here for error
        
        bj_x = -(L.L3*n[2]) / denom
        bj_y = -(S3*L.L3*n[2]) / denom
        bj_z = h + ((n[0] + S3 * n[1]) * L.L3) / denom

        A = -(bj_x + S3 * bj_y + 2 * L.L2) / bj_z
        B = (bj_x**2 + bj_y**2 + bj_z**2 + L.L1**2 - L.L0**2 - L.L2**2)/(2*bj_z)
        C = A**2 + 4
        D = 2*(A*B + 2*L.L2)
        E = B**2 + L.L2**2 - L.L1**2
        
        pj_x = (-D - sqrt(D**2 - 4*C*E))/(2*C)
        pj_y = S3*pj_x
        pj_z = sqrt(L[1]**2 - 4*pj_x**2 - 4*L[2]*pj_x - L[2]**2)
        
        # check elbow up vs elbow down
        if bj_z < reference_height:
            pj_z = -pj_z
            
        # adjust for physical build (0 -> vertical)
        radial = sqrt(pj_x**2 + pj_y**2)
        theta_raw_3 = degrees(atan2(radial - L.L2,pj_z))
        return self._ANGLE_OFFSET - theta_raw_3
    
if __name__ == "__main__":
    robot = Robot()