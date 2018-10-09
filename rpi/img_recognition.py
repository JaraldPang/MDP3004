import cv2 as cv2
import numpy
from picamera import PiCamera
from picamera.array import PiRGBArray
from Queue import queue
from timeit import default_timer as timer

#for debugging only
import matplotlib

class ImageProcessor()

	def __init__(self):
		self.jobs = queue()
		self.camera = PiCamera()
		self.camera.resolution = (1920,1080)

	def capture(pipe_endpoint):
		try:
			while 1:
				start = timer()
				img_name = pipe_endpoint.recv()
				print("Capturing...")
				rawCaptureHigh = PiRGBArray(camera, size=(1920,1080))
        		camera.capture(rawCaptureHigh,format='bgr', use_video_port=True)
        		pipe_endpoint.send("Captured")
        		end = timer()
        		print("Time taken for {} : {}".format(img_name, end - start))
        		cv2.imwrite('capture/image{}.jpg'.format(n),rawCaptureHigh.array)
				print("Terminating Capture...")
		finally:
			pass

	def identify(pipe_endpoint,bt_endpoint):
		print("Identifying...")
		print("Terminating identification...")
		#arrowFound aka arrFound
		bt_endpoint.write("arrFound{}")
		pass