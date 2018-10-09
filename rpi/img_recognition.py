import cv2 as cv2
import numpy
from picamera import PiCamera
from picamera.array import PiRGBArray
from Queue import queue

#for debugging only
import matplotlib

class ImageProcessor()

	def __init__(self):
		self.jobs = queue()
		self.camera = PiCamera()

	def capture(endpoint):
		print("Capturing...")
		rawCaptureHigh = PiRGBArray(camera, size=(1920,1080))
        camera.capture(rawCaptureHigh,splitter_port=1,format='bgr', use_video_port=True, resize=(1920,1080))
        cv2.imwrite('capture/image{}.jpg'.format(n),rawCaptureHigh.array)
		print("Terminating Capture...")


	def identify(pc_endpoint):
		print("Identifying...")
		print("Terminating identification...")
		#arrowFound aka arrFound
		pc_endpoint.write("arrFound{}")
		pass