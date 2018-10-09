import cv2 as cv
import numpy
from picamera import PiCamera
from picamera.array import PiRGBArray
from Queue import queue

#for debugging only
import matplotlib

class ImageProcessor():

	def __init__(self):
		self.jobs = queue()
		self.camera = PiCamera()
		# get sample image, used to check likeliness of arrow later
		ret1, th1 = cv.threshold(gray1, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
		cnts1 = cv.findContours(th1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		cnts1 = cnts1[1]

	def capture(endpoint):
		print("Capturing...")
		rawCaptureHigh = PiRGBArray(camera, size=(1920,1080))
		camera.capture(rawCaptureHigh,splitter_port=1,format='bgr', use_video_port=True, resize=(1920,1080))
		cv2.imwrite('capture/image{}.jpg'.format(n),rawCaptureHigh.array)
		print("Terminating Capture...")


	def identify(self, pc_endpoint):
		print("Identifying...")
		print("Terminating identification...")
		#arrowFound aka arrFound
		arrows = []
		arrowLoc = []
		#init sample
		# get sample image, used to check likeliness of arrow later
		img = cv.imread('testbed/arrow_real.jpg', cv.IMREAD_UNCHANGED)
		gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
		#get captured
		# image preprocessing
		blur = cv.GaussianBlur(capturedImage, (5, 5), 2)
		gray1 = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
		self.getImageLocation(gray, gray1, arrows)
		if arrows:
			self.getArrowLocatiion(arrows, robotLocation, robotDir, arrowLoc)
		#getImageLoc(sample, captured
		#if(getImageLoc has arrow
			pc_endpoint.write("arrFound{}")
		pass

	def CameraDir(robotDir):
		cameraSwitch = {
			"U" : "R",
			"R" : "D",
			"D" : "L",
			"L" : "U"
		}

	def getArrowLocation(self, arrows, robotLocation, robotDir, arrowLocArray):
		arrowLoc = []
		dirMatrix = [[0, 1], [1, 0], [0, -1], [-1, 0]]
		arrowSwitch = {
			"U": ["D"],
			"R": ["L"],
			"D": ["U"],
			"L": ["R"]
		}
		cameraSwitch = {
			"U": ["R", dirMatrix[1]],
			"R": ["D", dirMatrix[2]],
			"D": ["L", dirMatrix[3]],
			"L": ["U", dirMatrix[0]]
		}
		robotDirSwitch = {
			"U": ["U", dirMatrix[0]],
			"R": ["R", dirMatrix[1]],
			"D": ["D", dirMatrix[2]],
			"L": ["L", dirMatrix[3]]
		}
		robotDir = robotDirSwitch(robotDir, "No such direction")
		cameraDir = cameraSwitch(robotDir[0], "No such direction")
		dir = arrowSwitch(cameraDir[0])
		for arrow in arrows:
			if arrow[1] == "center":
				arrowLoc[0] = robotLocation[0] + (arrow[0] + 2)*cameraDir[1][0]
				arrowLoc[1] = robotLocation[1] + (arrow[0] + 2)*cameraDir[1][1]
			elif arrow[1] == "right":
				arrowLoc[0] = robotLocation[0] + (arrow[0] + 2) * cameraDir[1][0] - robotDir[1][0]
				arrowLoc[1] = robotLocation[1] + (arrow[0] + 2) * cameraDir[1][1] - robotDir[1][1]
			elif arrow[1] == "left":
				arrowLoc[0] = robotLocation[0] + (arrow[0] + 2) * cameraDir[1][0] + robotDir[1][0]
				arrowLoc[1] = robotLocation[1] + (arrow[0] + 2) * cameraDir[1][1] + robotDir[1][1]
			arrowLocArray.append([arrowLoc, dir])

	def getImageLocation(self, sampleImg, actualImage, arrows):
		#get sample image contours
		ret, th = cv.threshold(actualImage, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
		cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		cnts = cnts[1]
		# get sample image contours
		ret1, th1 = cv.threshold(sampleImg, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
		cnts1 = cv.findContours(th1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		cnts1 = cnts1[1]
		#get image size
		imgX = actualImage.shape[1]
		imgY = actualImage.shape[0]
		imgArea = imgX*imgY
		#for each contour found
		for (i, c) in enumerate(cnts):
			#find number of edges the object has
			peri = cv.arcLength(c, True)
			approx = cv.approxPolyDP(c, 0.01 * peri, True)
			#find area of the object
			objectArea = float(cv.contourArea(c))
			#find ratio of the area of object to the area of whole image
			objectAreaRatio = objectArea/imgArea

			#conditional check
			#if contour matches any of the conditions we are looking for, we append the distance and location
			#condition: 6-8 edges; matches given shape up to 0.25 likeliness; object to image area ratio
			if (6 <= len(approx) <= 8 and cv.matchShapes(cnts1[0], c, 1, 0.0) < 0.25):
				# find X-axis of contour
				M = cv.moments(c)
				if M["m00"] != 0:
					cx = int(M["m10"] / M["m00"])
				# find which part of the image the contour is on
				if imgX / 3 < cx < 2 * (imgX / 3):
					a = "center"
				elif cx < imgX / 3:
					a = "left"
				else:
					a = "right"
				if objectAreaRatio > 0.127:
					arrows.append((0, a))
				elif objectAreaRatio > 0.055:
					arrows.append((1, a))
				elif objectAreaRatio > 0.027:
					arrows.append((2, a))
				elif objectAreaRatio > 0.015:
					arrows.append((3, a))
