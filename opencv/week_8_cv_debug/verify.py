import cv2 as cv
import numpy
import time
import os
from timeit import default_timer as timer


def main():
    f = open("log.txt","w")
    print("Starting Arrow Recognition...")
    reference_img = cv.imread('reference_arrow.jpg', cv.IMREAD_GRAYSCALE)
    ret, th = cv.threshold(reference_img, 0, 255, cv.THRESH_BINARY)
    cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    #for image in os.listdir('photos'):
    for image in ['2,1,r.jpg']:
        robot_status = image[:-4]
        print("Processing {}...".format(robot_status))
        robot_x,robot_y,robot_dir = robot_status.split(",")
        start = timer()
        arrowsWithPartition = getImageLocation(cnts,robot_status)
        end = timer()
        print("Time taken for arrow analysis of {}: {}. Arrows Found: {}".format(robot_status,end - start, len(arrowsWithPartition)))
        if(arrowsWithPartition):
            arrowLocAndFace = getArrowLocation(arrowsWithPartition,(int(robot_x),int(robot_y)),robot_dir)
            for entry in arrowLocAndFace:
                print("FOUND! For Status: {}, writing to endpoint arrow location: {}".format(robot_status,entry))
                f.write("FOUND! For Status: {}, writing to endpoint arrow location: {}\n".format(robot_status,entry))

    f.close()
    print("Terminating identification...")


def getArrowLocation(arrows, robotLocation, robotDir):
    arrowLocArray = []
    #robot relative direction
    dirMatrix = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    arrowSwitch = {
        "u": "d",
        "r": "l",
        "d": "u",
        "l": "r"
    }
    cameraSwitch = {
        "u": ["r", dirMatrix[1]],
        "r": ["d", dirMatrix[2]],
        "d": ["l", dirMatrix[3]],
        "l": ["u", dirMatrix[0]]
    }
    robotDirSwitch = {
        "u": ["u", dirMatrix[0]],
        "r": ["r", dirMatrix[1]],
        "d": ["d", dirMatrix[2]],
        "l": ["l", dirMatrix[3]]
    }
    robotDir = robotDirSwitch[robotDir]
    cameraDir = cameraSwitch[robotDir[0]]
    dir = arrowSwitch[cameraDir[0]]
    for arrow in arrows:
        arrowLoc = []
        if arrow[1] == "center":
            arrowLoc.append(str(robotLocation[0] + (arrow[0] + 2) * cameraDir[1][0]))
            arrowLoc.append(str(robotLocation[1] + (arrow[0] + 2) * cameraDir[1][1]))
        elif arrow[1] == "right":
            arrowLoc.append(str(robotLocation[0] + (arrow[0] + 2) * cameraDir[1][0] - robotDir[1][0]))
            arrowLoc.append(str(robotLocation[1] + (arrow[0] + 2) * cameraDir[1][1] - robotDir[1][1]))
        elif arrow[1] == "left":
            arrowLoc.append(str(robotLocation[0] + (arrow[0] + 2) * cameraDir[1][0] + robotDir[1][0]))
            arrowLoc.append(str(robotLocation[1] + (arrow[0] + 2) * cameraDir[1][1] + robotDir[1][1]))
        arrowLoc.append(dir)
        arrowLocArray.append(','.join(arrowLoc))
    #list of strings with x,y,face
    return arrowLocArray

def getImageLocation(reference_contours, captured_image_location):
    arrows = []
    captured_image = cv.imread("photos/{}.jpg".format(captured_image_location), cv.IMREAD_UNCHANGED)
    gray = cv.cvtColor(captured_image, cv.COLOR_RGB2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 2)
    ret, thresholded_img = cv.threshold(gray, 50, 255, cv.THRESH_BINARY)
    cv.imwrite("debug/{}_th.jpg".format(captured_image_location),thresholded_img)
    captured_cnts = cv.findContours(thresholded_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    # get image size
    imgX = captured_image.shape[1]
    imgY = captured_image.shape[0]
    imgArea = imgX * imgY
    # for each contour found
    for (i, c) in enumerate(captured_cnts):
        # find number of edges the object has
        peri = cv.arcLength(c, True)
        approx = cv.approxPolyDP(c, 0.01 * peri, True)
        # find area of the object
        objectArea = float(cv.contourArea(c))
        # find ratio of the area of object to the area of whole image
        objectAreaRatio = objectArea / imgArea

        # conditional check
        # if contour matches any of the conditions we are looking for, we append the distance and location
        # condition: 6-8 edges; matches given shape up to 0.25 likeliness; object to image area ratio
        if (6 <= len(approx) <= 8 and cv.matchShapes(reference_contours[0], c, 1, 0.0) < 0.05):
            cv.drawContours(captured_image, [c], -1, (0,255,0), 3)
            cv.imwrite("debug/{}_cnt.jpg".format(captured_image_location),captured_image)
            # find X-axis of contour
            M = cv.moments(c)
            cx = int(M["m10"] / M["m00"])
            # find which part of the image the contour is on
            if imgX / 3 < cx < 2 * (imgX / 3):
                a = "center"
            elif cx < imgX / 3:
                a = "left"
            else:
                a = "right"
            #appends (distance in grid, image)    
            if objectAreaRatio > 0.127:
                print("{}, {}".format(len(approx),cv.matchShapes(reference_contours[0], c, 1, 0.0)))
                arrows.append((0, a))
            elif objectAreaRatio > 0.055:
                print("{}, {}".format(len(approx),cv.matchShapes(reference_contours[0], c, 1, 0.0)))
                arrows.append((1, a))
            elif objectAreaRatio > 0.027:
                print("{}, {}".format(len(approx),cv.matchShapes(reference_contours[0], c, 1, 0.0)))
                arrows.append((2, a))
            elif objectAreaRatio > 0.015:
                print("{}, {}".format(len(approx),cv.matchShapes(reference_contours[0], c, 1, 0.0)))
                arrows.append((3, a))


    return arrows


if __name__ == '__main__':
    main()
