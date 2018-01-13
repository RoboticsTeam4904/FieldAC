import cv2
import numpy as np
from ContourFinding import filterContours, filterContoursFancy
from SpikeFinding import findCenter
import config, WebCam, GripRunner, autocalibrate, Printing
def main():
	lastAngle = 0
	# WebCam.set(exposure=config.exposure, resolution=config.resolution, contrast=config.contrast, gain=config.gain)
	# autocalibrate.calibrate()
	config.resolution = WebCam.getResolution()
	config.degPerPxl = np.divide(config.nativeAngle, config.resolution)
	if not config.edited:
		GripRunner.editCode()
	if config.display:
		cv2.namedWindow("Contours Found")
	frameNum = 1
	while True:
		# if NetworkTabling.checkForCalibrate():
			# print "CALIBRATING the camera due to button press"
			# autocalibrate.calibrate()
			# NetworkTabling.putCalibrated()

		image = WebCam.getImage()
		contours = GripRunner.run(image)
		targets = filterContours(contours)
		isVisible, angleToGoal, distance = findCenter(targets)
		if lastAngle != 0 and not isVisible:
			angleToGoal = lastAngle
		else:
			lastAngle = angleToGoal
		if config.debug:
			Printing.printResults(contours=contours, distance=distance, angleToGoal=angleToGoal, isVisible=isVisible)
		if config.save and frameNum % 50 == 0:
			Printing.save(image)
		if config.display:
			Printing.drawImage(image, contours, targets)
			Printing.display(image)
		if config.save and frameNum % 50 == 0:
			Printing.save(image, withGrip=True)
		if config.display:
			Printing.display(image)
		# try:
		# 	NetworkTabling.publishToTables(isVisible=isVisible, angleToGoal=angleToGoal, distance=distance, frameNum=frameNum)
		# except Exception as error:
		# 	if config.debug:
		# 		print error
		frameNum += 1
	if config.display:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
