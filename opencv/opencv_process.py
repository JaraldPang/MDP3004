import numpy
import cv2 as cv
from matplotlib import pyplot as plt
from timeit import default_timer as timer


def main():

	face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
	eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
	img = cv.imread('sachin.jpg')
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

	faces = face_cascade.detectMultiScale(gray, 1.3, 5)

	for (x,y,w,h) in faces:
		cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = img[y:y+h, x:x+w]

		cv.imshow('img',img)
		cv.waitKey(0)
		cv.destroyAllWindows()

#required
if __name__ == '__main__':
	main()
    print("Exiting...")