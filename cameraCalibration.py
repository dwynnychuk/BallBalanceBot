import numpy as np
import glob
import cv2 as cv
import os
import matplotlib.pyplot as plt
import time

class Calibration:
    def __init__(self):
        self.fp = os.path.join(os.getcwd(), "calibration/")
        self.nrow = 9
        self.ncol = 6
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.defaultCalNum = 20
        
    def take_calibration_images(self):
        # Open camera
        fp1 = os.getcwd() + "/calibration/"
        cap = cv.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("Cannot open camera")
        
        time.sleep(1)
        for i in range(self.defaultCalNum):
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("Cannot access frame")
            cv.imshow(f"{i}",frame)
            cv.waitKey(100)
            time.sleep(2)
            cv.imwrite(fp1 + f"calibrationmac{i}.jpg",frame)
            
        cap.release()
        cv.destroyAllWindows()
    
    def calibrate_camera(self, showAll = True, printAll = True):
        worldpointscur = np.zeros((self.nrow*self.ncol, 3), np.float32)
        worldpointscur[:,:2] = np.mgrid[0:self.nrow, 0:self.ncol].T.reshape(-1,2)
        worldpointlist = []
        imagepointlist = []
        images = glob.glob(os.path.join(self.fp, "*.jpg"))
        for imgPath in images:
            imgBGR = cv.imread(imgPath)
            imgGr = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
            cornersFound, cornersOrg = cv.findChessboardCorners(imgGr,(self.nrow, self.ncol), None)
            
            if cornersFound == True:
                worldpointlist.append(worldpointscur)
                cornersRefined = cv.cornerSubPix(imgGr,cornersOrg,(11,11),(-1,-1), self.criteria)
                imagepointlist.append(cornersRefined)
                if showAll:
                    cv.drawChessboardCorners(imgGr,(self.nrow, self.ncol), cornersRefined, cornersFound)  
                    cv.imshow("frame", imgGr)
                    cv.waitKey(1000)          
        cv.destroyAllWindows()
        
        # Calibrate
        er, camMat, distCoefs, rVec, tVec = cv.calibrateCamera(worldpointlist,imagepointlist,imgGr.shape[::-1], None,None)
        
        # Print Values
        print(f"Representation Error:\n{er}\n")
        print(f"Camera Matrix:\n{camMat}\n")
        print(f"Distortion Coefficients:\n{distCoefs}\n")
        if printAll:
            print(f"Rotation Vector:\n{rVec}\n")
            print(f"Translation Vector:\n{tVec}")
        
if __name__ == "__main__":
    calibrate = Calibration()
    calibrate.take_calibration_images()
    time.sleep(5)
    calibrate.calibrate_camera()