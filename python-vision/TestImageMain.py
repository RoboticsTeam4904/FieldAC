import cv2
import numpy as np
from ContourFinding import filterContoursFancy
from SpikeFinding import findSpike
import GripRunner
import config
import NetworkTabling
if config.debug:
	import Printing

def main():
	if not config.edited:
		GripRunner.editCode()
	if config.display:
		cv2.namedWindow("Contours Found")
	image = cv2.imread(config.sampleImage)
	config.resolution = image.shape[1], image.shape[0]
	config.degPerPxl = np.divide(config.nativeAngle, config.resolution)
	contours = GripRunner.run(image)
	targets = filterContoursFancy(contours, image=image)
	isVisible, angleToGoal, distance = findSpike(targets)
	if config.debug:
		Printing.printResults(contours=contours, distance=distance, angleToGoal=angleToGoal, isVisible=isVisible)
	if config.save or config.display:
		Printing.drawImage(image, contours, targets)
		if config.save:
			Printing.save(image)
		if config.display:
			Printing.display(image)
	try:
		NetworkTabling.publishToTables(isVisible=isVisible, angleToGoal=angleToGoal, distance=distance)
	except Exception as error:
		if config.debug:
			print error
	if config.display:
		cv2.waitKey(0)
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
