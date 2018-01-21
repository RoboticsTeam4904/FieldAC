import cv2
camera = cv2.VideoCapture(1)
retval, image = camera.read()
cv2.imshow("img", image)
cv2.waitKey(0)