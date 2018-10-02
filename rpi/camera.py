from picamera import PiCamera
from timeit import default_timer as timer
from time import sleep

def main():
	
    camera = PiCamera()
    try:
        #camera.resolution = (3280,2464)
        #camera.shutter_speed = 600
        # Camera warm-up time
        camera.start_preview()
        start = timer()
        camera.capture('auto/image.jpg', use_video_port=True)
        end = timer()
        print("Time taken: {}".format(end-start))
        camera.stop_preview()
    finally:
        camera.close()
			

if __name__ == '__main__':
    main()