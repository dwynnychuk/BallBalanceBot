import cv2
import numpy as np

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("cannot open camera")
    exit()
    
while True:
    ret, frame = cap.read()
    
    if not ret:
        print("cannot return frame")
        break
    
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame', grey)
    if cv2.waitKey(1) == ord("q"):
        break
    
cap.release()
cv2.destroyAllWindows()