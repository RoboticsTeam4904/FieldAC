import cv2, copy
import numpy as np
import Printing
import config


numTargets = 2

sizeWeight = 1
ratioWeight = 0.5
rotationWeight = 0.05
rectangularWeight = 3
areaWeight = 0.5
quadWeight = 5
weights = np.array([sizeWeight, ratioWeight, rotationWeight, rectangularWeight, areaWeight, quadWeight])

maxArea, minArea = 300000, 400

def filterMinArea(contours, min_area=400):
	filtered_contours = []
	for contour in contours:
		area = cv2.contourArea(contour, False)
		if min_area < area:
			filtered_contours.append(contour)
	return filtered_contours

def filterContours(contours): # Find 2 largest contours.
	numContours = len(contours)
	if numContours > 1:
		largest_contour, second_largest_contour, largest_area, second_largest_area = None, None, 0, 0
		for i in range(numContours):
			temp_area = cv2.contourArea(contours[i], False)
			if temp_area > second_largest_area:
				if temp_area > largest_area:
					largest_contour, second_largest_contour = contours[i], largest_contour
					largest_area, second_largest_area = temp_area, largest_area
				else:
					second_largest_contour = contours[i]
					second_largest_area = temp_area
		return [largest_contour]#, second_largest_contour
	else:
		return contours

def filterContoursFancy(contours, image=None):
	if len(contours) == 0:
		return []

	numContours = len(contours)
	areas = np.array([cv2.contourArea(contour) for contour in contours])

	boundingRects = [cv2.boundingRect(contour) for contour in contours]
	widths, heights, positions = boundingInfo(boundingRects)

	rotatedRects = [cv2.minAreaRect(contour) for contour in contours]
	if config.withOpenCV3:
		rotatedBoxes = [np.int0(cv2.boxPoints(rect)) for rect in rotatedRects]
	else:
		rotatedBoxes = [np.int0(cv2.cv.BoxPoints(rect)) for rect in rotatedRects]
	rotatedAreas = [cv2.contourArea(box) for box in rotatedBoxes]

	sizeScores = [size(area)for area in areas]
	ratioScores = ratios(widths, heights)
	rotationScores = [rotation(rect) for rect in rotatedRects]
	rectangularScores = [distToPolygon(contour, poly) for contour,poly in zip(contours, rotatedBoxes)]
	areaScores = polygonAreaDiff(areas, rotatedAreas)
	quadScores = [Quadrify(contour) for contour in contours]

	rectangularScores = np.divide(rectangularScores, widths)

	scores = np.array([sizeScores, ratioScores, rotationScores, rectangularScores, areaScores, quadScores])
	contourScores = np.dot(weights, scores)

	correctInds, incorrectInds = sortedInds(contourScores)
	correctContours = np.array(contours)[correctInds]

	if config.extra_debug:
		print "size, ratio, rotation, rectangular, area, quad"
		print "Weights:", weights
		print "Scores: ", contourScores
		print np.average(scores, axis=1)
		if len(incorrectInds) != 0:
			print "AVG, WORST", test(scores, correctInds, incorrectInds)
		for i in range(numContours):
			print "CONTOUR " + str(i)
			print np.multiply(scores[:, i], weights) #newWeights
			print contourScores[i]
			if image:
				img = copy.deepcopy(image)
				Printing.drawImage(img, contours[:i] + contours[i+1:], contours[i], False)
				Printing.display(img, "contour " + str(i), doResize=True)
			cv2.waitKey(0)
		cv2.destroyAllWindows()
	return correctContours

def sortedInds(scores):
	sortedScoresIndices = scores.argsort()
	return sortedScoresIndices[:numTargets], sortedScoresIndices[numTargets:]

def test(scores, correctInds, incorrectInds):
	correct = scores[:, correctInds]
	incorrect = scores[:, incorrectInds]

	worstCorrect = np.amax(correct, axis=1)
	bestIncorrect = np.amin(incorrect, axis=1)
	worstdiffs = np.divide(bestIncorrect+0.00001, worstCorrect+0.00001) - 1

	avgCorrect = np.average(correct, axis=1)
	avgIncorrect = np.average(incorrect, axis=1)
	avgdiffs = np.divide(avgIncorrect+0.00001, avgCorrect+0.00001) - 1

	return avgdiffs, worstdiffs

def boundingInfo(rects):
	rects = np.array(rects)
	widths = rects[:,2]
	heights = rects[:,3]
	positions = rects[:,:2]
	return widths, heights, positions

def distToPolygon(contour, polygon):
	tests = [cv2.pointPolygonTest(polygon, (point[0][0], point[0][1]), True) for point in contour]
	return np.average(np.absolute(tests))

def rotation(rotatedRect): # not super good
	angle = rotatedRect[2]
	rotation = np.minimum(np.add(angle, 90), np.negative(angle)) #That's just how minarearect works
	return rotation

def size(area): # Too large bad, too small bad
	diff = 1
	if area > maxArea:
		diff = np.divide(area, maxArea)
	if area < minArea:
		diff = np.divide(minArea, area)
	return np.log(diff)


# The dimensions of the tape is 2 x 5 inches, so expect ed height is 1.5 times the width	
def ratios(widths, heights):
	ratios = np.divide(np.true_divide(heights, widths), 1.5)
	return np.absolute(np.log(ratios))

def polygonAreaDiff(areas, polyAreas):
	ratios = np.divide(polyAreas, areas)
	return np.absolute(np.log(ratios))

def Quadrify(contour):
	epsilon = 10
	for i in range(1,10):
		quad = cv2.approxPolyDP(contour, epsilon, True)
		length = len(quad)
		randomVar = np.random.random()
		epsilon = np.multiply(epsilon, np.true_divide(np.add(length, randomVar), np.add(4, randomVar)))
		# print epsilon, length
		if length == 4:
			return np.multiply(i, 0.01)
	return 1


def filterContoursAutocalibrate(contours, image=None):
	if len(contours) == 0:
		return []

	numContours = len(contours)
	areas = np.array([cv2.contourArea(contour) for contour in contours])

	boundingRects = [cv2.boundingRect(contour) for contour in contours]
	widths, heights, positions = boundingInfo(boundingRects)

	rotatedRects = [cv2.minAreaRect(contour) for contour in contours]
	if config.withOpenCV3:
		rotatedBoxes = [np.int0(cv2.boxPoints(rect)) for rect in rotatedRects]
	else:
		rotatedBoxes = [np.int0(cv2.cv.BoxPoints(rect)) for rect in rotatedRects]
	rotatedAreas = [cv2.contourArea(box) for box in rotatedBoxes]

	sizeScores = [size(area)for area in areas]
	ratioScores = ratios(widths, heights)
	rotationScores = [rotation(rect) for rect in rotatedRects]
	rectangularScores = [distToPolygon(contour, poly) for contour,poly in zip(contours, rotatedBoxes)]
	areaScores = polygonAreaDiff(areas, rotatedAreas)
	quadScores = [Quadrify(contour) for contour in contours]

	rectangularScores = np.divide(rectangularScores, widths)

	scores = np.array([sizeScores, ratioScores, rotationScores, rectangularScores, areaScores, quadScores])
	contourScores = np.dot(weights, scores)

	correctInds, incorrectInds = sortedInds(contourScores)
	correctContours = np.array(contours)[correctInds]

	averageScore = 0
	for i in range(numContours):
		averageScore += sizeScores[i]
		averageScore +=  ratioScores[i]
		averageScore +=  rotationScores[i]
		averageScore +=  rectangularScores[i]
		averageScore +=  areaScores[i]
		averageScore +=  quadScores[i]
	averageScore /= numContours
	return averageScore
