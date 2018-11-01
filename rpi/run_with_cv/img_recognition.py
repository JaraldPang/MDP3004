import cv2 as cv
import numpy
import time
import sys
from picamera import PiCamera
from queue import Queue
from timeit import default_timer as timer

#for debugging only
#import matplotlib

class ImageProcessor():
    
    def __init__(self):
        self.jobs = Queue()

    #sequence as i verb sequence
    def sequence_images(self,listener_endpoint_pc):
        dir = sys.path[0]
        while 1:
            img_name = listener_endpoint_pc.recv()
            start = timer()
            time.sleep(0.05)
            yield "{}/capture/{}.jpg".format(dir,img_name)
            end = timer()
            #if(end-start < 0.1):
            #   time.sleep(0.1-(end-start))
            #   print("Camera too fast: {}. Slept for: {}".format(end-start,0.1-(end-start)))
            listener_endpoint_pc.send("Captured")
            self.jobs.put(img_name)
            print("Time taken for sequence capturing {} : {}.".format(img_name, end - start))

    def capture(self,listener_endpoint_pc):
        camera = PiCamera(resolution=(1920,1080))
        try:
            dir = sys.path[0]
            print("Starting Capture Thread...")
            camera.capture_sequence(self.sequence_images(listener_endpoint_pc), use_video_port=True)
            print("Terminating Capture...")
        finally:
            camera.close()

    def capture_old(self,listener_endpoint_pc):
        camera = PiCamera(resolution=(1920,1080))
        try:
            dir = sys.path[0]
            print("Starting Capture Thread...")
            while 1:
                img_name = listener_endpoint_pc.recv()
                start = timer()
                camera.capture("{}/capture/{}.jpg".format(dir,img_name), use_video_port=True)
                end = timer()
                listener_endpoint_pc.send("Captured")
                print("Time taken for single capturing {} : {}".format(img_name, end - start))
                self.jobs.put(img_name)
            print("Terminating Capture...")
        finally:
            camera.close()


    def identify(self, listener_endpoint_rpi):
        print("Starting Arrow Recognition Thread...")
        reference_img = cv.imread('{}/reference_arrow.jpg'.format(sys.path[0]), cv.IMREAD_GRAYSCALE)
        ret, th = cv.threshold(reference_img, 0, 255, cv.THRESH_BINARY)
        cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        while 1:
            if(self.jobs.empty() is False):
                robot_status = self.jobs.get()
                robot_x,robot_y,robot_dir = robot_status.split(",")
                start = timer()
                arrowsWithPartition = self.getImageLocation(cnts,robot_status)
                end = timer()
                print("Time taken for arrow analysis of {}: {}. Arrows Found: {}".format(robot_status,end - start, len(arrowsWithPartition)))
                if(arrowsWithPartition):
                    arrowLocAndFace = self.getArrowLocation(arrowsWithPartition,(int(robot_x),int(robot_y)),robot_dir)
                    for entry in arrowLocAndFace:
                        print("FOUND! For Status: {}, writing to endpoint arrow location: {}".format(robot_status,entry))
                        listener_endpoint_rpi.send("arrfound{}".format(entry))
                else:
                    #test
                    listener_endpoint_rpi.send("NOT FOUND")
                    continue
            else:
                time.sleep(0.5)

        print("Terminating identification...")


    def getArrowLocation(self,arrows, robotLocation, robotDir):
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

    def getImageLocation(self,reference_contours, captured_image_location):
        arrows = []
        dir = sys.path[0]
        captured_image = cv.imread("{}/capture/{}.jpg".format(dir,captured_image_location), cv.IMREAD_UNCHANGED)
        gray = cv.cvtColor(captured_image, cv.COLOR_RGB2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 2)
        ret, thresholded_img = cv.threshold(blur, 50, 255, cv.THRESH_BINARY)
        #cv.imwrite("capture/{}_th.jpg".format(captured_image_location),thresholded_img)
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
            if(not(6 <= len(approx) <= 8 )):
                continue

            # conditional check
            # if contour matches any of the conditions we are looking for, we append the distance and location
            # condition: 6-8 edges; matches given shape up to 0.25 likeliness; object to image area ratio
            if (cv.matchShapes(reference_contours[0], c, 1, 0.0) < 0.1):
                #cv.drawContours(captured_image, [c], -1, (0,255,0), 3)
                #cv.imwrite("{}/capture/{}_cnt.jpg".format(dir,captured_image_location),captured_image)
                # find area of the object
                objectArea = float(cv.contourArea(c))
                # find ratio of the area of object to the area of whole image
                objectAreaRatio = objectArea / imgArea
                
                #appends (distance in grid, image)
                
                if 0.105 < objectAreaRatio < 0.31:
                    # find X-axis of contour
                    #M = cv.moments(c)
                    #cx = int(M["m10"] / M["m00"])
                    # find which part of the image the contour is on
                    #if imgX / 3 < cx < 2 * (imgX / 3):
                    #    a = "center"
                    #elif cx < imgX / 3:
                    #    a = "left"
                    #else:
                    #    a = "right"
                    print("0cm: {}".format(objectAreaRatio))
                    a = "center"
                    arrows.append((0, a))
                elif 0.048 < objectAreaRatio < 0.092:
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
                    print("10cm: {}".format(objectAreaRatio))                        
                    arrows.append((1, a))
                elif 0.026 < objectAreaRatio < 0.041:
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
                    print("20cm: {}".format(objectAreaRatio)) 
                    arrows.append((2, a))
                elif 0.015 < objectAreaRatio < 0.023:
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
                    arrows.append((3, a))
                    print("30cm: {}".format(objectAreaRatio))
                else:
                    print("No Match: {}".format(objectAreaRatio))

        return arrows


