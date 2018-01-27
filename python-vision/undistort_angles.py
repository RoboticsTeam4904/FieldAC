import numpy as np
import cv2
import sys


img = cv2.imread(fname)
imshape = img.shape

def calculateDistortionMatrix(corners, row, col, imshape):
    objp = np.zeros((cbrow*cbcol,3), np.float32)
    objp[:,:2] = np.mgrid[0:cbcol,0:cbrow].T.reshape(-1,2)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    # dist = np.array([-0.13615181, 0.53005398, 0, 0, 0]) # no translation
    h, w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

def boardCoords(corner_lines):
    u, v 
    # find the tile, then use the tile's corner edges to calculate percentages along u and v vectors (maybe use both edges and draw a line changing continuously with percentages along top and bottom edge)
    # coord = tile corner coord + percentage of tile coords

def undistortCorners(corners, distortion_mtx, dst):
    cv2.undistortPoints(src, cameraMatrix, distCoeffs)

def calcBoardDists(corners_board_coords, corners_x_and_y_board_coords):
    board_dists = np.zeros_like(corners)
    for i,j
        dist_x = np.distance(corners[i,j], corners_x_and_y[i,j][0])
        dist_y = np.distance(corners[i,j], corners_x_and_y[i,j][1])
        board_dists[i,j] = [dist_x, dist_y]
    np.multiply(side_length, board_dists)



corners, row, col = findCorners(img)

distortion_mtx = calculateDistortionMatrix(corners, row, col, imshape)
corners_x_and_y = [([corner[0], 0], [0, corner[1]]) for corner in corners]
corners_undistorted = undistortCorners(corners, distortion_mtx, dst)
corners_x_and_y_undistorted = undistortCorners(corners_x_and_y, distortion_mtx, dst)
corner_lines = undistortCorners(corners, row, col, imshape)

corners_board_coords = calcBoardCoords(corners) = [[[i,j] for j in range(len(corners[i]))] for i in range(len(corners))]
corners_x_and_y_board_coords = calcBoardCoords(corners_x_and_y)
board_dists = calcBoardDists(corners_board_coords, corners_x_and_y_board_coords)
true_dists = np.zeros_like(corners) #array of floats

def calcAngle(a, b, c):
    return arccos((a^2 + b^2 - c^2)/(2*a*b))

def calcDist(theta, a, b):
    return sqrt(a^2 + b^2 - 2*a*b*cos(theta))

width = side_length*row
height = side_length*col
top_left_to_right_angle = calcAngle(dist_top_left, width, dist_top_right)
top_left_to_down_angle = calcAngle(dist_top_left, height, dist_bottom_left)
bottom_right_to_left_angle = calcAngle(dist_bottom_right, width, dist_bottom_left)
bottom_right_to_up_angle = calcAngle(dist_bottom_right, height, dist_top_right)

# make sure you know which way the coordinates go (indeces order vs top right and such)
for i in range(1,row-1): # Don't include corners
    true_dists[i,0] = calcDist(top_left_to_right_angle, row*i, dist_top_left) # need new angles for inner grid dists
    true_dists[1,col-1]
for j in range(1,col-1):
    true_dists[0,j]
    true_dists[row-1,j]

accurate_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) #not linear?
pixel_coords = np.reshape(accurate_corners, (row,col))

 
dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
corner_lines = np.reshape(corners, (row,col))


def findCorners(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    for i in range(1, 50): #caps out at 50x50 board
        for j in range(i):
            row = 2 + i - j
            col = 3 + j
            ret, corners = cv2.findChessboardCorners(gray, (row,col), None)
            if ret:
                return corners, (row,col)
    print "could not find chessboard"
    return False


def convertToBoardDists(u, v, corner):




rotation = calcRotation(corner_lines)





def undistort(cbrow=sys.argv[1], cbcol=sys.argv[2], fileName=sys.argv[3]): # number of black corners in row and column on chessboard picture
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cbrow = int(cbrow)
    cbcol = int(cbcol)
    print(cbrow,cbcol)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cbrow*cbcol,3), np.float32)
    objp[:,:2] = np.mgrid[0:cbcol,0:cbrow].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    fname = fileName #"/Users/niksure/Documents/workspace/2018-Field/python-vision/TestImages/TEST0.jpg"
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray,(cbrow,cbcol),None)
    print ret
    cv2.imshow("img",img)
    cv2.waitKey(500)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        # print corners2
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (cbrow,cbcol), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    dist = np.array([-0.13615181, 0.53005398, 0, 0, 0]) # no translation
    h, w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # # undistort
    # dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # # crop the image
    # x,y,w,h = roi
    # dst = dst[y:y+h, x:x+w]
    # cv2.imwrite('calibresult.png',dst)
    # undistort
    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
    # print(dst)

    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('calibresult.png',dst)

undistort()
