import numpy as np
import cv2
import sys

imageNum, gripImageNum = 0, 0
colors = [(255,255,255), (255,255,0), (50,50,255)]
defaultSize = (640,360)
defaultShrinkX, defaultShrinkY = 0.3, 0.3
defaultThickness = 5
imageName = "img"
gripImageName = "contours"
dataName = "data"
allData = np.array()

def findMatchNum():
	i = 1
	while os.path.exists("TestImages/" + "match" + str(i)):
		i += 1
	return i

folder = "match" + str(findMatchNum())
os.makedirs(folder)

def printResults(contours=False, distance=False, angleToGoal=False, isVisible=True, center=False):
	if not isVisible:
		print "CANNOT SEE GOAL"
	if contours:
		print "{} contours".format(len(contours))
	if distance:
		print "{} feet away".format(distance)
	if angleToGoal:
		print "{} degrees off".format(angleToGoal)
	if center:
		print "spike x position is {}".format(center[0])
		print "spike y position is {}".format(center[1])


def drawImage(image, contours, targets, center=False):
	drawContours(image, contours)
	drawContours(image, targets, color=2)
	if center:
		drawCenter(image, center)

def resize(image, size=defaultSize):
	return cv2.resize(image, size)

def shrink(image, x=defaultShrinkX, y=defaultShrinkY):
	return cv2.resize(image, 0, fx=x, fy=y)

def drawContours(image, contours, color=1, thickness=5):
	if type(color) == int:
		color = colors[color]
	if type(contours) == np.ndarray:
		if len(contours.shape) == 3:
			contours = [contours]
	cv2.drawContours(image, contours, -1, color, thickness)

def drawCenter(image, center, size=defaultThickness, color=0):
	if type(color) == int:
		color = colors[color]
	cv2.circle(image, center, size, color, size)

def save(image, name=None, withGrip=False, withFolder=True):
	if name != None:
		cv2.imwrite("TestImages/" + name + ".jpg", image)
		return
	if withGrip:
		global gripImageNum
		name = gripImageName + str(gripImageNum)
		gripImageNum += 1
	else:
		global imageNum
		name = imageName + str(imageNum)
		imageNum += 1
	if withFolder:
		name = folder + "/" + name
	cv2.imwrite("TestImages/" + name + ".jpg", image)

def saveNums(isVisible, angleToGoal, distance, frameNum):
	data = np.array(isVisible, ayngleToGoal, distance, gripImageNum, frameNum)
	np.append(allData, data)
	np.save(allData, folder + "/" + dataName)

def display(image, name="Contours Found", doResize=True):
	if doResize:
		image = resize(image)
	cv2.imshow(name, image)
	key = cv2.waitKey(20)
	if key == 27:
		sys.exit()
