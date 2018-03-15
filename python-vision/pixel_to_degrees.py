import numpy as np
import matplotlib as pyplot
import random

# dataFile = np.load("data.npz")
divConst = 4

#tests
pxCoords = np.array([[random.randint(1, 1080), random.randint(1,1080)] for i in range(28)])
thetas = np.array([[random.randint(1, 60), random.randint(1,60)] for i in range(28)])
np.savetxt(fname='thetas.csv', X=thetas, delimiter=';')
np.savetxt(fname='pxCoords.csv', X=pxCoords, delimiter=';')

print(thetas)

# #takes in numpy savez file
# def PolyFit3D(pixel_coords, thetas):
# 	# pixel_coords, thetas = file['px'], file['thetas']
# 	theta_xs = np.array([thetas[0][idx][0] for idx in range(len(thetas))]) 
# 	theta_ys = np.array([thetas[0][idx][1] for idx in range(len(thetas))])
# 	numPts = pixel_coords.shape[0]
# 	errors_x = []
# 	errors_y = []
# 	#x_polynomial = 
# 	for polyDeg in range(numPts // divConst):
# 		polynomial, error = matlab.polyFitn([pixel_coords], theta_xs, polyDeg)
# 		polyn2sympoly(polynomial) 