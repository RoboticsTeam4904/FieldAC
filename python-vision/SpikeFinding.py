import cv2
import numpy as np
import config

def findSpike(contours): # returns isVisible, angleToGoal, distance
	isVisible=False
	numContours = len(contours)
	if numContours == 0:
		return False, 0, 0
	contour = np.concatenate(contours)
	X,Y,W,H = cv2.boundingRect(contour)
	center = (np.add(X, np.divide(W,2)), np.add(Y, np.true_divide(H,2)))
	print np.degrees(np.multiply(config.degPerPxl[0], np.subtract(np.true_divide(config.resolution[0], 2), 0)))
	angleToGoal = np.multiply(config.degPerPxl[0], np.subtract(np.true_divide(config.resolution[0], 2), center[0]))
	if numContours == 2:
		isVisible = True
		x1,y1,w1,h1 = cv2.boundingRect(contours[0]) #possibly change to make more resistant to small anomalies at the top of the contour
		x2,y2,w2,h2 = cv2.boundingRect(contours[1])
		d1, d2 = distanceFromHeight(y1), distanceFromHeight(y2)
		if x1 > x2:
			d1, d2 = d2, d1
		distance = trueDistance(d1, d2)
		if not distance: # Something went wrong with mounting or detection.
			isVisible = False
		phi = angle(distance, d2)
		robotAngle = np.add(phi, angleToGoal)
		x, y = np.multiply(distance, np.cos(phi)), np.multiply(distance, np.sin(phi))
		if config.debug:
			print "distance                      ", distance, distanceFromHeight(Y)
			print "x,y,angle     ", x, y, np.degrees(angleToGoal)
	else:
		distance = distanceFromHeight(Y)
	return isVisible, np.degrees(angleToGoal), distance

def distanceFromHeight(y):
	degrees = np.multiply(config.degPerPxl[1], np.subtract(np.true_divide(config.resolution[1], 2), y))
	degrees = np.add(degrees, config.cameraTilt)
	distance = np.divide(config.displacement, np.tan(degrees))
	return distance

def trueDistance(d1, d2):
	squares = np.add(np.square(d1), np.square(d2))
	squared = np.subtract(np.multiply(2, squares), np.square(config.width))
	if squared < 0:
		return False # Something went wrong
	d = np.divide(np.sqrt(squared), 2)
	return d
	# d = 1/2 * sqrt(2*(d1^2+d2^2)-w^2)

def angle(d, d2):
	squares = np.subtract(np.add(np.true_divide(np.square(config.width), 4), np.square(d)), np.square(d2))
	phi = np.arccos(np.divide(squares, np.multiply(config.width, d)))
	return np.subtract(np.pi, phi)
	# angle = pi - acos(1/4*w^2 + d^2 - d2^2 / wd)
