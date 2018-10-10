import cv2 as cv
import numpy
from picamera import PiCamera
from picamera.array import PiRGBArray
from queue import Queue
from timeit import default_timer as timer

#for debugging only
#import matplotlib

class ImageProcessor():
    
    def __init__(self):
        self.jobs = Queue()

    def capture(self,listener_endpoint):
        try:
            camera = PiCamera(resolution=(1920,1080))
            print("Starting Capture Thread...")
            while 1:
                img_name = listener_endpoint.recv()
                print("Capturing...")
                start = timer()
                camera.capture("capture/{}.jpg".format(img_name), use_video_port=True)
                #listener_endpoint.send("Captured")
                end = timer()                
                print("Time taken for {} : {}".format(img_name, end - start))
                self.jobs.put(img_name)
            print("Terminating Capture...")
        finally:
            pass

    #old capture
    def capture_old(self,listener_endpoint):
        try:
            camera = PiCamera()
            camera.resolution = (1920,1080)
            print("Starting Capture Thread...")
            rawCapture = PiRGBArray(camera, size=(1920,1080))
            while 1:
                img_name = listener_endpoint.recv()
                print("Capturing...")
                start = timer()
                camera.capture(rawCapture,format='bgr', use_video_port=True)
                #listener_endpoint.send("Captured")
                cv2.imwrite('capture/{}.jpg'.format(img_name),rawCapture.array)
                rawCapture.truncate(0)
                end = timer()                
                print("Time taken for {} : {}".format(img_name, end - start))
                self.jobs.put(img_name)
            print("Terminating Capture...")
        finally:
            pass


    def identify(self, pc_endpoint):
    	print("Starting Arrow Recognition Thread...")
        reference_img = cv.imread('reference_arrow.jpg', cv.IMREAD_GRAYSCALE)
        ret, th = cv.threshold(referenceImg, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        while 1:
        	if(self.jobs.empty() is False):
        		robot_status = self.jobs.get()
        		robot_x,robot_y,robot_dir = robot_status.split(",")
        		arrowsWithPartition = self.getImageLocation(cnts,robot_status)
        		if(arrowsWithPartition):
        			continue
        		else:
	        		arrowLocAndFace = self.getArrowLocation(arrowsWithPartition,(robot_x,robot_y),robot_dir)
	        		for entry in arrowLocAndFace:
	        			pc_endpoint.write("arrFound{}".format(entry))

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
        captured_image = cv2.imread("capture/{}.jpg".format(captured_image_location), cv.IMREAD_GRAYSCALE)
        blur = cv.GaussianBlur(capturedImage, (5, 5), 2)
        gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
        ret, thresholded_img = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        captured_cnts = cv.findContours(thresholded_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        # get image size
        imgX = actualImage.shape[1]
        imgY = actualImage.shape[0]
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
            if (6 <= len(approx) <= 8 and cv.matchShapes(reference_contours, c, 1, 0.0) < 0.25):
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
                    arrows.append((0, a))
                elif objectAreaRatio > 0.055:
                    arrows.append((1, a))
                elif objectAreaRatio > 0.027:
                    arrows.append((2, a))
                elif objectAreaRatio > 0.015:
                    arrows.append((3, a))

        return arrows


