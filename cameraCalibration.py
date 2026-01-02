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
        self.camera_fov = (1280, 720)
        
        # see if running on raspberry pi
        try:
            from picamera2 import Picamera2
            self.Pi_Camera_Available = True
            self.picam2 = Picamera2()
        except:
            self.Pi_Camera_Available = False                
        
    def take_calibration_images(self):
        # Open camera
        os.makedirs(self.fp, exist_ok=True)
        
        if self.Pi_Camera_Available:
            config = self.picam2.create_video_configuration(
                main={"size": self.camera_fov}
                )
            self.picam2.configure(config)
            self.picam2.start()
            get_frame = lambda: cv.cvtColor(
                self.picam2.capture_array(),
                cv.COLOR_RGB2BGR
                )
            print("Using Raspberry Pi Camera for Calibration")
        else:
            cap = cv.VideoCapture(0)
            if not cap.isOpened():
                raise RuntimeError("Cannot open camera")
            get_frame = lambda: cap.read()[1]
            print("Using Mac Camera for Calibration")
        
        time.sleep(1)
        
        try: 
            for i in range(self.defaultCalNum):
                print(f"Capturing Image: {i}")
                time.sleep(0.5)
                frame = get_frame()
                if frame is None:
                    continue
                
                #cv.imshow(f"{i}",frame)
                cv.waitKey(100)
                
                print(f"Captured image: {i}, saving...")
                
                cv.imwrite(self.fp + f"calibration_{i}.jpg",frame)
                print(f"Saved image: {i}")
                time.sleep(2)            
                
        finally:
            if self.Pi_Camera_Available:
                self.picam2.stop()
            else:
                cap.release()
        
            cv.destroyAllWindows()
                
    def calibrate_camera(self, showAll = True, printAll = True):
        worldpointscur = np.zeros((self.nrow*self.ncol, 3), np.float32)
        worldpointscur[:,:2] = np.mgrid[0:self.nrow, 0:self.ncol].T.reshape(-1,2)

        worldpointlist = []
        imagepointlist = []
        
        images = glob.glob(os.path.join(self.fp, "*.jpg"))
        
        if not images:
            raise RuntimeError("No Calibration Images Found")
        
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
        
        if self.Pi_Camera_Available:
            np.savez(
                self.fp + "pi_camera_calibration.npz",
                camera_matrix = camMat,
                distortion_coefficients = distCoefs,
                image_size = imgGr.shape[::-1]
                )
        
        if printAll:
            print(f"Rotation Vector:\n{rVec}\n")
            print(f"Translation Vector:\n{tVec}")
        
if __name__ == "__main__":
    calibrate = Calibration()
    calibrate.take_calibration_images()
    time.sleep(5)
    calibrate.calibrate_camera()