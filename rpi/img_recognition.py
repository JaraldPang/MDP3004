import cv2 as cv2
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
                cv2.imwrite('capture/image{}.jpg'.format(img_name),rawCapture.array)
                rawCapture.truncate(0)
                end = timer()                
                print("Time taken for {} : {}".format(img_name, end - start))
                
            print("Terminating Capture...")
        finally:
            pass

    def identify(self, pipe_endpoint,bt_endpoint):
        print("Identifying...")
        print("Terminating identification...")
        #arrowFound aka arrFound
        bt_endpoint.write("arrFound{}")
        pass