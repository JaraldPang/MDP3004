import numpy
import cv2 as cv
import imutils
from matplotlib import pyplot as plt

def main():

    img = cv.imread('testbed/image20cm.jpg', cv.IMREAD_UNCHANGED)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, th = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    img1 = cv.imread('testbed/arrow_real.jpg', cv.IMREAD_UNCHANGED)
    gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
    ret1, th1 = cv.threshold(gray1, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    cnts1 = cv.findContours(th1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts1 = cnts1[0] if imutils.is_cv2() else cnts1[1]


    # loop over the contours
    for (i, c) in enumerate(cnts):
        x, y, w, h = cv.boundingRect(c)
        if (cv.matchShapes(cnts1[0], c, 1, 0.0) < 0.02):
            cv.drawContours(img, [c], -1, (0, 255, 0), 3)
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    #cv.imwrite('testbed/results.jpg', img)

    plt.imshow(img,'gray')
    plt.show()

if __name__ == '__main__':
    main()