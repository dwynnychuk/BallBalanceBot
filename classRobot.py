from logger import get_logger
from math import sqrt

logger = get_logger(__name__)

class Robot:
    def __init__(self):
        self.L = [0.1, 0.1, 0.1, 0.1]   # populate with CAD
    
    def kinematics_fwd(self, angles: list[float]):
        pass
    
    def kinematics_inv(self, nVec: list[float], h: float) -> list[float]:
        # Arm 01
        b1x = None      # Need to finish substitution
        b1y = 0
        b1z = h - (nVec[0]*self.L[3])/sqrt(nVec[2]**2+nVec[0]**2)
        
        # Arm 02
        b2x = (self.L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] - sqrt(3)*nVec[1])**2)
        b2y = -(sqrt(3)*self.L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] - sqrt(3)*nVec[1])**2)
        b2z = h - ((nVec[0] - sqrt(3)*nVec[1])*self.L[3])/sqrt((nVec[0] - sqrt(3)*nVec[1])**2 - 4*nVec[2]**2)
        
        # Arm 03
        b3x = (self.L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] + sqrt(3)*nVec[1])**2)
        b3y = (sqrt(3)*self.L[3]*nVec[2])/sqrt(4*nVec[2]**2 - (nVec[0] + sqrt(3)*nVec[1])**2)
        b3z = h - ((nVec[0] + sqrt(3)*nVec[1])*self.L[3])/sqrt((nVec[0] + sqrt(3)*nVec[1])**2 - 4*nVec[2]**2)