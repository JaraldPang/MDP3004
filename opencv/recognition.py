import numpy
import cv2 as cv
from matplotlib import pyplot as plt
import multiprocessing
import picamera

def check_image(ref,compare):

	#reference image
	gray1 = cv.cvtColor(ref, cv.COLOR_BGR2GRAY)
	ret1, th1 = cv.threshold(gray1, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
	cnts1 = cv.findContours(th1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	cnts1 = cnts1[0] if imutils.is_cv2() else cnts1[1]

	#supplied image
	blur = cv.GaussianBlur(compare, (5, 5), 2)
	gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
	ret, th = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
	cnts = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]

	numImg = 0
	for (i, c) in enumerate(cnts):
		x, y, w, h = cv.boundingRect(c)
		peri = cv.arcLength(c, True)
		approx = cv.approxPolyDP(c, 0.015 * peri, True)
		hull = cv.convexHull(c)
		hullArea = float(cv.contourArea(hull))
		if (6 <= len(approx) <= 8 and hullArea > 10000 and cv.matchShapes(cnts1[0], c, 1, 0.0) < 0.1):
			numImg += 1
			#cv.drawContours(compare, [c], -1, (0, 255, 0), 3)
			#cv.rectangle(compare, (x, y), (x + w, y + h), (0, 255, 0), 2)

	if(numImg == 1):
		return True

	return False


def main():
	reference_img = cv.imread('testbed/arrow_real.jpg', cv.IMREAD_UNCHANGED)
	camera = picamera.Camera()

	num_of_found_arrows = 0;
	check_image(reference_img)



