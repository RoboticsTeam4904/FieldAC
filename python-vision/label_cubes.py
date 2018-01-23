import cv2
import numpy as np
import ContourFinding
import SpikeFinding
import config, WebCam, GripRunner, autocalibrate, Printing

# returns list of length len(img_names) with each element as a list of the bounding boxes of cubes in that image
def label(img_names):
	for img_name in img_names:
		print img_name
		img = cv2.imread(img_name)
		width, height, _ = img.shape
		contours = GripRunner.run(img)
		filtered_contours = ContourFinding.filterArea(contours)
		boxes = np.array([cv2.boundingRect(contour) for contour in filtered_contours]) #X,Y,W,H
		print boxes.shape
		X, Y, W, H = boxes[:,0], boxes[:,1], boxes[:,2], boxes[:,3]
		X, Y = np.add(X, np.true_divide(W,2)), np.add(Y, np.true_divide(H,2))
		X, Y, W, H = np.divide(X, width), np.divide(Y, height), np.divide(W, width), np.divide(H, height)
		bounding_boxes.append([X,Y,W,H])

if __name__ == '__main__':
	img_names = ['TestImages/TEST' + str(i) + '.jpg' for i in range(1,14)]
	print label(img_names)
