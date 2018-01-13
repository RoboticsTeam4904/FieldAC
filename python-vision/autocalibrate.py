import numpy as np

import GripRunner, Printing, WebCam
from config import debug, display, resolution
import cv2, time
from ContourFinding import *

minExposure = 1
maxExposure = 50
resolutionArea = np.multiply(resolution[0], resolution[1])
maxArea = np.divide(resolutionArea, 4)

targetAverage = 30
averageThreshold = 10

numTests = 2
maxBrightnessIterations = 400


def oldDecrepidAndFrailCalibrate():
	exposure = WebCam.getExposure()
	if debug:
		b = time.clock()

	for iteration in xrange(maxBrightnessIterations):
		for k in range(10):
			image = WebCam.getImage()
		if display:
			Printing.display(image)

		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		value = cv2.split(image)[2]
		average = cv2.mean(value)[0]
		print "average brightness is ", average
		if np.absolute(np.subtract(average, targetAverage)) < averageThreshold:
			if debug:
				print iteration, "num iterations, brightness lowering"
			break

		scaleBy = np.divide(targetAverage, average)
		newExposure = np.minimum(np.maximum(np.multiply(exposure, scaleBy), minExposure), maxExposure)
		if exposure == newExposure == minExposure:
			break
		exposure = newExposure
		WebCam.set(exposure=exposure)
	if debug:
		print exposure, "bright", time.clock() - b

	numGoodFrames = 0
	if debug:
		s = time.clock()
	for i in range(1000):
		print "Sleeping!"
		for i in range(10):
			image = WebCam.getImage()
		time.sleep(0)
		image = WebCam.getImage()
		contours = GripRunner.run(image)
		numContours = len(contours)
#		print numContours, exposure, WebCam.getExposure()
		if i%50 == 0:
			print "iteration #", i, "  Number of contours seen: ", numContours
		if numContours != 0:
			if tooLarge(contours):
				exposure = np.divide(exposure, 10)
		print "num contours", numContours
		if numContours == 2:
			numGoodFrames += 1
			if numGoodFrames == numTests:
				if debug:
					print i, "num iterations grip"
					print "Finished grip succesfully", time.clock() - s
				return True
		else:
			numGoodFrames = 0
		randomVar = np.random.random_sample()
		scaleBy = np.true_divide(2+randomVar, numContours+randomVar)/3
		print "old exposure: ", exposure
		exposure = np.minimum(np.maximum(np.multiply(exposure, scaleBy), minExposure), maxExposure)
		print "new exposure: ", exposure
		WebCam.set(exposure=exposure)
		if display:
			Printing.drawContours(image, contours)
			Printing.display(image)
	if debug: 
		print "Failed grip", time.clock() - s
	return False

def calibrate():
	maxScore = 0
	maxScoreExposure = 0
	for exposure in range(1, 60):
		WebCam.set(exposure=exposure)
		image = WebCam.getImage()
		contours = GripRunner.run(image)
		averageScore = filterContoursAutocalibrate(contours, image)
		if averageScore > maxScore:
			maxScore = averageScore
			maxScoreExposure = exposure
	WebCam.set(exposure=exposure)
	return True


def tooLarge(contours):
	areas = [cv2.contourArea(contour, False) for contour in contours]
	largest = np.amax(areas)
	if largest > maxArea:
		return True
	else:
		return False

def displace():
	WebCam.set(exposure=2000)


def test():
	displace()
	start = time.clock()
	newCalibrate()
	exposure = WebCam.getExposure()
	print time.clock() - start, "TOTAL TIME"

	while display:
		image = WebCam.getImage()
		contours = GripRunner.run(image)
		Printing.drawContours(image, contours)
		Printing.display(image)
		cv2.waitKey(20)

# Get average value at the end of test to recalibrate targetAverage
	# image = cv2.imread('TestImages/Cancer.jpg')
	# image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	# value = cv2.split(image)[2]
	# # value = np.array([image[:,:,2]])
	# average = cv2.mean(value)
	# print average