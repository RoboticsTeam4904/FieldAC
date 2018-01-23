import cv2
import numpy as np
import time
import os
from grip_edited import GripPipeline

pipeline = GripPipeline()

def filterMinArea(contours, min_area=400):
	filtered_contours = []
	for contour in contours:
		area = cv2.contourArea(contour, False)
		if min_area < area:
			filtered_contours.append(contour)
	return filtered_contours

# returns list of length len(img_names) with each element as a list of the bounding boxes of cubes in that image
def label(img_names, debug=False):
	bounding_boxes = []
	img_times = []
	for img_name in img_names:
		start_time = time.time()
		if debug:
			print img_name
		img = cv2.imread(img_name)
		height, width, _ = img.shape
		if debug:
			print width, height
		pipeline.process(img)
		contours = pipeline.filter_contours_output
		filtered_contours = filterMinArea(contours, min_area=width*height/2500.0)
		if len(filtered_contours) == 0:
			bounding_boxes.append(np.array([]))
			continue
		boxes = np.array([cv2.boundingRect(contour) for contour in filtered_contours]) #X,Y,W,H
		X, Y, W, H = boxes[:,0], boxes[:,1], boxes[:,2], boxes[:,3]
		if debug:
			print X, Y, W, H
		X, Y = np.add(X, np.true_divide(W,2)), np.add(Y, np.true_divide(H,2))
		X, Y, W, H = np.true_divide(X, width), np.true_divide(Y, height), np.true_divide(W, width), np.true_divide(H, height)
		if debug:
			print X, Y, W, H
		boxes = np.rot90(np.array([X,Y,W,H]))
		bounding_boxes.append(boxes)
		img_times.append(time.time()-start_time)
	print np.average(img_times)
	if debug:
		print bounding_boxes
	return bounding_boxes

def labelFolder(folder_name):
	img_names = os.listdir(folder_name)
	label(img_names)

if __name__ == '__main__':
	img_names = ['TestImages/TEST' + str(i) + '.jpg' for i in range(1,14)]
	# img_names = os.listdir(folder_name)
	bounding_boxes = label(img_names)
