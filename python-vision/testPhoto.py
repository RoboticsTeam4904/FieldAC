import WebCam, Printing, GripRunner, ContourFinding, SpikeFinding, autocalibrate
n = 0
def d():
	while True:
		o()

def s():
	global n
	image = WebCam.getImage()
	Printing.save(image, name="TEST" + str(n))
	n += 1

def e(exposure):
	WebCam.set(exposure=exposure)

def o():
	image = WebCam.getImage()
	contours = GripRunner.run(image)
	targets = ContourFinding.filterContours(contours) # To be edited if the last filter is changed in case of algorithmic changes.
	# center, distance = SpikeFinding.findCenterandDist(targets) #if 2, join and find center, if 1, return val, if 0 return input. if adjustCoords:	center[0] -= halfWidth
	# # Printing.printResults(contours, center, distance)
	Printing.drawImage(image, contours, targets)
	Printing.display(image)

def a():
	autocalibrate.calibrate()
	print WebCam.getExposure()
