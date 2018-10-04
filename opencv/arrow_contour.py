import numpy
import cv2 as cv
import imutils
from matplotlib import pyplot as plt

def main():

    numImg = 0
    cam = cv.VideoCapture(0)
    cam.set(3, 1280)
    cam.set(4, 720)

    while True:
        _, img = cam.read()

        img1 = cv.imread('testbed/arrow_real.jpg', cv.IMREAD_UNCHANGED)
        gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
        ret1, th1 = cv.threshold(gray1, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        cnts1 = cv.findContours(th1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts1 = cnts1[0] if imutils.is_cv2() else cnts1[1]

        blur = cv.GaussianBlur(img, (5, 5), 2)
        gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
        ret, th = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        for (i, c) in enumerate(cnts):
            x, y, w, h = cv.boundingRect(c)
            peri = cv.arcLength(c, True)
            approx = cv.approxPolyDP(c, 0.015 * peri, True)
            hull = cv.convexHull(c)
            hullArea = float(cv.contourArea(hull))
            if (6 <= len(approx) <= 8 and hullArea > 10000 and cv.matchShapes(cnts1[0], c, 1, 0.0) < 0.1):
                numImg += 1
                cv.drawContours(img, [c], -1, (0, 255, 0), 3)
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
				
        print(numImg, " Arrows")
        numImg = 0
        cv.imshow('', img)
        cv.waitKey(1)

if __name__ == '__main__':
    main()