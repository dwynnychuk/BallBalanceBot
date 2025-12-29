from logger import get_logger
from math import sqrt, degrees, atan2

logger = get_logger(__name__)

class Robot:
    def __init__(self):
        # TODO populate with CAD, mechanical constants
        self.L = [0.07, 0.07, 0.046, 0.076]
        logger.debug("Robot class initialized")
    
    def kinematics_inv(self, nVec: list[float], h: float) -> list[float]:
        """Inputs:
        nVec: normal vector [alpha, beta, gamma]
        h: desired height 
        
        Constants:
        L: [L1, L2, L3, LPlate]
        
        Calculated Values
        bj: Ball joint
        """
        L = self.L
        angle_offset = 90
        
        # reference position (theta 0)
        A = (L[1] + L[2])/h
        B = (L[0]**2 + h**2 - L[3]**2 - (L[1] + L[2])**2)/(2*h)
        C = A**2 + 1
        D = 2*(A*B - (L[1] + L[2]))
        E = B**2 + (L[1] + L[2])**2 - L[0]**2
        PX = (-D + sqrt(D**2 - 4*C*E))/(2*C)
        PZ = sqrt(L[0]**2 - (PX - (L[1] + L[2]))**2)
        
        # Arm 01
        bj_denom_1 = sqrt(nVec[0]**2 + nVec[2]**2)
        bj1x = (L[3]*nVec[2])/bj_denom_1
        bj1y = 0
        bj1z = h - (nVec[0]*L[3])/bj_denom_1
        
        A_1 = (L[2] - bj1x)/bj1z
        B_1 = (bj1x**2 + bj1y**2 + bj1z**2 + L[1]**2 - L[0]**2 - L[2]**2) / (2*bj1z)
        C_1 = A_1**2 + 1
        D_1 = 2*(A_1*B_1 - L[2])
        E_1 = B_1**2 + L[2]**2 - L[1]**2
        
        pj1x = (-D_1 + sqrt(D_1**2 - (4*C_1*E_1)))/(2*C_1)
        pj1y = 0
        pj1z = sqrt(L[1]**2 - (pj1x - L[2])**2)
        
        # check elbow up vs elbow down
        if bj1z < PZ:
            pj1z = -pj1z
        
        # adjust for physical build (0 -> vertical)
        theta_1 = angle_offset - degrees(atan2(pj1z, pj1x-L[2]))
        
        
        # Arm 02
        bj_denom_2 = sqrt(4*nVec[2]**2 + nVec[0]**2 + 3*nVec[1]**2 - 2*sqrt(3)*nVec[0]*nVec[1])
        bj2x = -(L[3]*nVec[2])/bj_denom_2
        bj2y = (sqrt(3)*L[3]*nVec[2])/bj_denom_2
        bj2z = h + ((nVec[0] - sqrt(3)*nVec[1])*L[3])/bj_denom_2
        
        A_2 = (sqrt(3)*bj2y - 2*L[2] - bj2x)/bj2z
        B_2 = (bj2x**2 + bj2y**2 + bj2z**2 + L[1]**2 - L[0]**2 - L[2]**2)/2*bj2z
        C_2 = A_2**2 + 4
        D_2 = 2*(A_2*B_2 + 2*L[2])
        E_2 = B_2**2 + L[2]**2 - L[1]**2
        
        pj2x = (-D_2 - sqrt(D_2**2 - 4*C_2*E_2))/(2*C_2)
        pj2y = -sqrt(3)*pj2x
        pj2z = sqrt(L[1]**2 - 4*pj2x**2 - 4*L[2]*pj2x - L[2]**2)
        
        # check elbow up vs elbow down
        if bj2z < PZ:
            pj2z = -pj2z
            
        # adjust for physical build (0 -> vertical)
        theta_2 = angle_offset - degrees(atan2(pj2z, (sqrt(pj2x**2 + pj2y**2) - L[2])))
    
        
        # Arm 03
        bj_denom_3 = sqrt(4*nVec[2]**2 + nVec[0]**2 + 2*sqrt(3)*nVec[0]*nVec[1] + 3*nVec[1]**2)
        bj3x = -(L[3]*nVec[2])/bj_denom_3
        bj3y = -(sqrt(3)*L[3]*nVec[2])/bj_denom_3
        bj3z = h + ((nVec[0] + sqrt(3)*nVec[1])*L[3])/bj_denom_3

        A_3 = -(bj3x + sqrt(3)*bj3y + 2*L[2])/bj3z
        B_3 = (bj3x**2 + bj3y**2 + bj3z**2 + L[1]**2 - L[0]**2 - L[2]**2)/(2*bj3z)
        C_3 = A_3**2 + 4
        D_3 = 2*(A_3*B_3 + 2*L[2])
        E_3 = B_3**2 + L[2]**2 - L[1]**2
        
        pj3x = (-D_3 - sqrt(D_3**2 - 4*C_3*E_3))/(2*C_3)
        pj3y = sqrt(3)*pj3x
        pj3z = sqrt(L[1]**2 - 4*pj3x**2 - 4*L[2]*pj3x - L[2]**2)
        
        # check elbow up vs elbow down
        if bj3z < PZ:
            pj3z = -pj3z
            
        # adjust for physical build (0 -> vertical)
        theta_3 = angle_offset - degrees(atan2(pj3z, (sqrt(pj3x**2 + pj3y**2) - L[2])))

        thetas = [theta_1, theta_2, theta_3]
        
        return thetas
    
    def control_platform(self, desired_pose: list) -> None:
        """control platform
        desired pose to come from PID output and include normal vector as well as height"""
        pass