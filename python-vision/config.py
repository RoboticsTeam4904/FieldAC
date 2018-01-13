import cv2
import numpy as np
# Logging/debug settings
# --------------
debug = False
extra_debug = False #step through each contour in the webcam by turning on extra_debug
save = True
display = False

# Camera settings
# --------------
exposure = 15
gain = 10
contrast = 50
displacement = (4.25 + 5)/12.0 # Vertical feet from camera to bottom of vision target + Height of target in feet
cameraTilt = 0
width = 8.25/12 #from centers. targets are 2x5 inches and 6.25 inches apart
resolution = (640, 480)
nativeAngle  = (np.radians(64), np.radians(48)) #experimentally determined 10 pxl per deg at 640x480, going down by a v smol amount at the edge of the frame
degPerPxl = np.divide(nativeAngle, resolution)

# Misc
# -------------
ip = "10.49.4.2"
team = 4904
withOpenCV3 = int(cv2.__version__[0]) == 3
edited = False
sampleImage = "TestImages/GearTest.png"
