import cv2
import numpy as np
from matplotlib import pyplot as plt

#template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY) #imread,0 is B/W.
oriImages=[]
testimages=[]
threshold = 0.75

cam=cv2.VideoCapture(0)
cam.set(3,1280)
cam.set(4,720)

while True:
    ret, tImg = cam.read()
    colortImg = tImg.copy()
    #tImg = cv2.resize(tImg, (0,0), fx=0.2, fy=0.2)
    tImg = cv2.cvtColor(tImg,cv2.COLOR_BGR2GRAY)
    tImg = cv2.blur(tImg, (7,7))
    #tImg = cv2.Canny(tImg,200,450)

    template = cv2.imread('arrow.jpg', 0)
    template = cv2.resize(template, (0,0), fx=0.3, fy=0.3)
    template = cv2.blur(template, (7,7)) #max 7,7
    #template = cv2.Canny(template,200,400)
    w, h = template.shape[::-1]


    res = cv2.matchTemplate(tImg,template,cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    print(min_val,max_val)
    if max_val < threshold :
        print("did not hit threshold")

    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(tImg, pt, (pt[0] + w, pt[1] + h), 255, 5)

    cv2.imshow('result', tImg)
    cv2.waitKey(1)
