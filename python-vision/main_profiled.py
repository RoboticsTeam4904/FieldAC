import time
init_time = time.time()
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
	iter_times = []
	print "----setup_time:", time.time()-init_time
	while True:
		# if NetworkTabling.checkForCalibrate():
			# print "CALIBRATING the camera due to button press"
			# autocalibrate.calibrate()
			# NetworkTabling.putCalibrated()
		iter_start_time = time.time()
		image = WebCam.getImage()
		print "----image_load_time:", time.time()-iter_start_time
		image_got_time = time.time()
		contours = GripRunner.run(image)
		print "----contour_finding_time:", time.time()-image_got_time
		contour_found_time = time.time()
		targets = filterContours(contours)
		print "----contour_filtering_time:", time.time()-contour_found_time
		# contour_filtered_time = time.time()
		isVisible, angleToGoal, distance = findCenter(targets)
		if lastAngle != 0 and not isVisible:
			angleToGoal = lastAngle
		else:
			lastAngle = angleToGoal
		iter_time = time.time()-iter_start_time
		print "----iter_" + str(frameNum) + "_time:", iter_time
		iter_times.append(iter_time)
		print "============avg_iter_time:" + str(np.average(iter_times)) + "============"

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
