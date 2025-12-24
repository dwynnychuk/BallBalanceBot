from logger import get_logger
from math import sqrt, degrees, atan2

logger = get_logger(__name__)

class Robot:
    def __init__(self):
        self.L = [0.1, 0.1, 0.1, 0.1]   # populate with CAD, mechanical constants
    
    def kinematics_fwd(self, angles: list[float]):
        pass
    
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
        
        # Arm 01
        bj1x = (L[3]*nVec[2])/sqrt(nVec[0]**2 + nVec[2]**2)
        bj1y = 0
        bj1z = h - (nVec[0]*L[3])/sqrt(nVec[2]**2+nVec[0]**2)
        
        A_1 = (L[2] - bj1x)/bj1z
        B_1 = (bj1x**2 + bj1y**2 + bj1z**2 + L[1]**2 - L[0]**2 - L[2]**2) / 2*bj1z
        C_1 = A_1**2 + 1
        D_1 = 2*(A_1*B_1 - L[2])
        E_1 = B_1**2 + L[2]**2 - L[1]**2
        
        pj1x = (-D_1 + sqrt(D_1**2 - (4*C_1*E_1)))/(2*C_1)
        pj1y = 0
        pj1z = sqrt(L[1]**2 - (pj1x - L[2])**2)
        
        theta_1 = degrees(atan2(pj1z, pj1x-L[2]))
        
        
        # Arm 02
        bj2x = (L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] - sqrt(3)*nVec[1])**2)
        bj2y = -(sqrt(3)*L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] - sqrt(3)*nVec[1])**2)
        bj2z = h - ((nVec[0] - sqrt(3)*nVec[1])*L[3])/sqrt((nVec[0] - sqrt(3)*nVec[1])**2 - 4*nVec[2]**2)
        
        A_2 = (sqrt(3)*bj2y + 2*L[2] - bj2x)/bj2z
        B_2 = (bj2x**2 + bj2y**2 + bj2z**2 + L[1]**2 - L[0]**2 - L[2]**2)/2*bj2z
        C_2 = A_2**2 + 4
        D_2 = 2*A_2*B_2 - 4*L[2]
        E_2 = B_2**2 + L[2]**2 - L[1]**2
        
        pj2x = (-D_2 - sqrt(D_2**2 - 4*C_2*E_2))/(2*C_2)
        pj2y = -sqrt(3)*pj2x
        pj2z = sqrt(L[1]**2 - 4*pj2x**2 + 4*L[2]*pj2x - L[2]**2)
        
        theta_2 = degrees(atan2(pj2z, (sqrt(pj2x**2 + pj2y**2) - L[2])))
    
        
        # Arm 03
        bj3x = (L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] + sqrt(3)*nVec[1])**2)
        bj3y = (sqrt(3)*L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] + sqrt(3)*nVec[1])**2)
        bj3z = h - ((nVec[0] + sqrt(3)*nVec[1])*L[3])/sqrt((nVec[0] + sqrt(3)*nVec[1])**2 - 4*nVec[2]**2)

        A_3 = 0
        B_3 = 0
        C_3 = 0
        D_3 = 0
        E_3 = 0
        
        pj3x = 0
        pj3y = 0
        pj3z = 0
        
        theta_3 = degrees(0)
