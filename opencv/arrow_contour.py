import numpy
import cv2 as cv
from matplotlib import pyplot as plt
import time

def main():
    t0 = time.time()
    arrows = []
    cam = cv.VideoCapture(0)
    cam.set(3, 1280)
    cam.set(4, 720)
    a = "none"

    #get sample image, used to check likeliness of arrow later
    img1 = cv.imread('testbed/arrow_real.jpg', cv.IMREAD_UNCHANGED)
    gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
    ret1, th1 = cv.threshold(gray1, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    cnts1 = cv.findContours(th1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts1 = cnts1[1]

    #read image
    img = cv.imread('test/image40c.jpg', cv.IMREAD_UNCHANGED)
    #image preprocessing
    blur = cv.GaussianBlur(img, (5, 5), 2)
    gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
    #get image size
    imgX = gray.shape[1]
    imgY = gray.shape[0]
    imgArea = imgX*imgY
    # draw division on image, 3 parts (left, right, center)
    cv.line(img, (int(imgX / 3), 0), (int(imgX / 3), 1080), (255, 0, 0), 2)
    cv.line(img, (int(2 * imgX / 3), 0), (int(2 * imgX / 3), 1080), (255, 0, 0), 2)
    #find contours in image
    ret, th = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[1]
    t0 = time.time()
    #for each contour found
    for (i, c) in enumerate(cnts):
        #get info to draw bounding rectagle
        x, y, w, h = cv.boundingRect(c)
        #find number of edges the object has
        peri = cv.arcLength(c, True)
        approx = cv.approxPolyDP(c, 0.01 * peri, True)
        #find area of the object
        objectArea = float(cv.contourArea(c))
        #find ratio of the area of object to the area of whole image
        objectAreaRatio = objectArea/imgArea

        #test for 10cm left and right (half arrow)
        # if (3 <= len(approx) <= 5 and 0.003 < objectAreaRatio < 0.06
        #         and a != "center" and cv.matchShapes(cnts1[0], c, 1, 0.0) < 2.5):
        #     print(cv.matchShapes(cnts1[0], c, 1, 0.0))
        #     arrows.append((0, a))
        #     cv.drawContours(img, [c], -1, (0, 255, 0), 3)
        #     cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #     cv.putText(img, a, (cx, cy), cv.FONT_HERSHEY_SIMPLEX,
        #                1.0, (255, 0, 0), 2)

        #conditional check
        #if contour matches any of the conditions we are looking for, we append the distance and location
        #condition: 6-8 edges; matches given shape up to 0.25 likeliness; object to image area ratio
        if (6 <= len(approx) <= 8 and cv.matchShapes(cnts1[0], c, 1, 0.0) < 0.25):
            # find X-axis of contour
            M = cv.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # find which part of the image the contour is on
            if imgX / 3 < cx < 2 * (imgX / 3):
                a = "center"
            elif cx < imgX / 3:
                a = "left"
            else:
                a = "right"
            if objectAreaRatio > 0.127:
                arrows.append((0, a))
                print(objectAreaRatio)
                cv.drawContours(img, [c], -1, (0, 255, 0), 3)
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.putText(img, a, (cx, cy), cv.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0), 2)
            elif objectAreaRatio > 0.055:
                arrows.append((10, a))
                print(objectAreaRatio)
                cv.drawContours(img, [c], -1, (0, 255, 0), 3)
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.putText(img, a, (cx, cy), cv.FONT_HERSHEY_SIMPLEX,
                           1.0, (255, 0, 0), 2)
            elif objectAreaRatio > 0.027:
                arrows.append((20, a))
                print(objectAreaRatio)
                cv.drawContours(img, [c], -1, (0, 255, 0), 3)
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.putText(img, a, (cx, cy), cv.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0), 2)
            elif objectAreaRatio > 0.015:
                arrows.append((30, a))
                print(objectAreaRatio)
                cv.drawContours(img, [c], -1, (0, 255, 0), 3)
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.putText(img, a, (cx, cy), cv.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0), 2)

    if arrows:
        print(arrows)

    cv.imwrite('houghlines3.jpg', img)
    # cv.imshow('', img)
    # cv.waitKey(1)

    t1 = time.time()
    print(t1-t0, "Seconds")

if __name__ == '__main__':
    main()