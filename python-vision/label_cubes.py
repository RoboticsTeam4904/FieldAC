import cv2
import numpy as np
import GripRunner, ContourFinding, Printing

min_area = 500

# returns list of length len(img_names) with each element as a list of the bounding boxes of cubes in that image
def label(img_names):
	bounding_boxes = []
	for img_name in img_names:
		# print img_name
		img = cv2.imread(img_name)
		height, width, _ = img.shape
		# print width, height
		contours = GripRunner.run(img)
		filtered_contours = ContourFinding.filterMinArea(contours, min_area=width*height/2500.0)
		boxes = np.array([cv2.boundingRect(contour) for contour in filtered_contours]) #X,Y,W,H
		X, Y, W, H = boxes[:,0], boxes[:,1], boxes[:,2], boxes[:,3]
		# print X, Y, W, H
		X, Y = np.add(X, np.true_divide(W,2)), np.add(Y, np.true_divide(H,2))
		X, Y, W, H = np.true_divide(X, width), np.true_divide(Y, height), np.true_divide(W, width), np.true_divide(H, height)
		# print X, Y, W, H
		boxes = np.rot90(np.array([X,Y,W,H]))
		bounding_boxes.append(boxes)
	# print bounding_boxes
	return bounding_boxes

if __name__ == '__main__':
	img_names = ['TestImages/TEST' + str(i) + '.jpg' for i in range(1,14)]
	print label(img_names)
