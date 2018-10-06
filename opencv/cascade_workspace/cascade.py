import numpy
import cv2 as cv
from matplotlib import pyplot as plt
from timeit import default_timer as timer


def main():

	arrow_cascade = cv.CascadeClassifier('cascade.xml')
	img = cv.imread('testbed/20cm.jpg')
	#gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

	arrows = arrow_cascade.detectMultiScale(img, 1.1, 5)
	#arrows = arrow_cascade.detectMultiScale(gray, 1.3, 5)

	for (x,y,w,h) in arrows:
		cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
		roi_color = img[y:y+h, x:x+w]

	plt.imshow(img,'gray')
	plt.show()
	#cv.imshow('img',img)
	#cv.waitKey(0)
	#cv.destroyAllWindows()

#required
if __name__ == '__main__':
	print("Starting")
	main()
	print("Exiting...") 