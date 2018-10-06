from picamera import PiCamera
from picamera.array import PiRGBArray
from timeit import default_timer as timer
from time import sleep
import cv2

def main():
	

    camera = PiCamera()
    camera.resolution=(1280,760)
    rawCaptureLow = PiRGBArray(camera, size=(1280,760))
    rawCaptureHigh = PiRGBArray(camera, size=(1920,1080))

    try:
        #camera.resolution = (3280,2464)
        #camera.shutter_speed = 600
        # Camera warm-up time
        #start = timer()
        n = 1
        while(1):
            rawCaptureLow.truncate(0)
            sleep(0.1)
            camera.capture(rawCaptureLow,splitter_port=0,format='bgr', use_video_port=True)
            #image = frame.array
            cv2.imshow("Frame", rawCaptureLow.array)
            key = cv2.waitKey(1)
            if(key == ord("q")):
                break
            if(key == ord("a")):
                camera.resolution=(1920,1080)
                camera.capture(rawCaptureHigh,splitter_port=1,format='bgr', use_video_port=True, resize=(1920,1080))
                cv2.imwrite('capture/image{}.jpg'.format(n),rawCaptureHigh.array)
                n = n + 1
                print("Captured")
                rawCaptureHigh.truncate(0)
                camera.resolution=(1280,760)
            #end = timer()
        #print("Time taken: {}".format(end-start))
    finally:
        camera.close()
			

if __name__ == '__main__':
    main()