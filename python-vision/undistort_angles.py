import numpy as np
import cv2
import sys
import math

# img = cv2.imread(fname)
# imshape = img.shape

def calculateDistortionMatrix(corners, row, col, imshape):
    objp = np.zeros((cbrow*cbcol,3), np.float32)
    objp[:,:2] = np.mgrid[0:cbcol,0:cbrow].T.reshape(-1,2)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    # dist = np.array([-0.13615181, 0.53005398, 0, 0, 0]) # no translation
    h, w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

# def boardCoords(corner_lines):
#     u, v 
    # find the tile, then use the tile's corner edges to calculate percentages along u and v vectors (maybe use both edges and draw a line changing continuously with percentages along top and bottom edge)
    # coord = tile corner coord + percentage of tile coords

def undistortCorners(corners, distortion_mtx, dst):
    cv2.undistortPoints(src, cameraMatrix, distCoeffs)

# def calcBoardDists(corners_board_coords, corners_x_and_y_board_coords):
#     board_dists = np.zeros_like(corners)
#     for i,j
#         dist_x = np.distance(corners[i,j], corners_x_and_y[i,j][0])
#         dist_y = np.distance(corners[i,j], corners_x_and_y[i,j][1])
#         board_dists[i,j] = [dist_x, dist_y]
#     np.multiply(side_length, board_dists)



# corners, row, col = findCorners(img)

# distortion_mtx = calculateDistortionMatrix(corners, row, col, imshape)
# corners_x_and_y = [([corner[0], 0], [0, corner[1]]) for corner in corners]
# corners_undistorted = undistortCorners(corners, distortion_mtx, dst)
# corners_x_and_y_undistorted = undistortCorners(corners_x_and_y, distortion_mtx, dst)
# corner_lines = undistortCorners(corners, row, col, imshape)

# corners_board_coords = calcBoardCoords(corners) = [[[i,j] for j in range(len(corners[i]))] for i in range(len(corners))]
# corners_x_and_y_board_coords = calcBoardCoords(corners_x_and_y)
# board_dists = calcBoardDists(corners_board_coords, corners_x_and_y_board_coords)
# true_dists = np.zeros_like(corners) #array of floats

def calcAngle(a, b, c):
    return arccos((a^2 + b^2 - c^2)/(2*a*b))

def calcDist(theta, a, b):
    return sqrt(a^2 + b^2 - 2*a*b*cos(theta))

#width = side_length*row
width = 1080
height = 1920
# height = side_length*col
# top_left_to_right_angle = calcAngle(dist_top_left, width, dist_top_right)
# top_left_to_down_angle = calcAngle(dist_top_left, height, dist_bottom_left)
# bottom_right_to_left_angle = calcAngle(dist_bottom_right, width, dist_bottom_left)
# bottom_right_to_up_angle = calcAngle(dist_bottom_right, height, dist_top_right)
center = np.array([width/2, height/2])

def findCenterTile():
    corner_coords, rowCols = undistort()
    cbrow, cbcol = rowCols
    dists = []
    coords = []
    x, y = center

    #finds corners that are above and to the left of the center
    for lineNum in range(len(corner_coords)):
        if corner_coords[lineNum][0][0] < y and corner_coords[lineNum][0][1] > x:
            print(corner_coords[lineNum][0][0], corner_coords[lineNum][0][1], lineNum)
            dists.append(np.linalg.norm(center - corner_coords[lineNum][0]))
            coords.append(lineNum)

    idx = dists.index(min(dists))
    coord = coords[idx]

    #pixel coords of corners
    top_left_corner = corner_coords[coord]
    top_right_corner = corner_coords[coord + cbrow]
    bottom_left_corner = corner_coords[coord + 1]
    bottom_right_corner = corner_coords[coord + cbrow + 1]

    tile_pixel_coords = np.array([top_left_corner, top_right_corner, bottom_left_corner, bottom_right_corner])
    boardCoords = np.array([[(coord + 1) // cbrow, (coord + 1) % cbrow], [(coord + cbrow + 1) // cbrow, (coord + cbrow + 1) % cbrow], [coord // cbrow, coord % cbrow], [(coord + cbrow) // cbrow, (coord + cbrow) % cbrow]])
    return tile_pixel_coords, boardCoords


def findCenterBoordCoords():
    pxCoords, bdCoords = findCenterTile()
    tl, tr, bl, br = pxCoords #topleft, topright, bottom left, bottom right
    bd_tl, bd_tr, bd_bl, bd_br = bdCoords
    u_1 = np.subtract(br, bl) 
    u_2 = np.subtract(tr, tl)

    #mathematica equation
    x = (1/(2*(-u_1[0][1]*u_2[0][0] + u_1[0][0]*u_2[0][1])))*(-center[1]*u_1[0][0] - tl[0][1]*u_1[0][0] - center[0]*u_1[0][1] + tl[0][0]*u_1[0][1] + bl[0][1]*u_2[0][0] + center[1]*u_2[0] - bl[0]*u_2[0][1] + center[0]*u_2[0][1] - math.sqrt((center[1]*u_1[0][0] + tl[0][1]*u_1[0][0] + center[0]*u_1[0][1] - tl[0][0]*u_1[0][1] - bl[0][1]*u_2[0][0] - center[1]*u_2[0][0] + bl[0][0]*u_2[0][1] - center[0]*u_2[0][1])**2 - 4*(bl[0][1]*center[0] + bl[0][0]*center[1] - bl[0][1]*tl[0][0] - center[1]*tl[0][0] + bl[0][0]*tl[0][1] - center[0]*tl[0][1])*(-u_1[0][1]*u_2[0][0] + u_1[0][0]*u_2[0][1])))
    p_1 = bl + x*u_1
    L = tl - bl + x*(u_2 - u_1)
    y_1 = (center[0] - p_1[0][0])/L[0][0]
    y_2 = (center[1] - p_1[0][1])/L[0][1]
 
    print(y_1, y_2)
    if y_1 != y_2:
        print("RIP calculations")

    center_board_coords = np.array([bd_bl[0] + x, y_1]) 

    print(center_board_coords)

# # make sure you know which way the coordinates go (indices order vs top right and such)
# for i in range(1,row-1): # Don't include corners
#     true_dists[i,0] = calcDist(top_left_to_right_angle, row*i, dist_top_left) # need new angles for inner grid dists
#     true_dists[1,col-1]
# for j in range(1,col-1):
#     true_dists[0,j]
#     true_dists[row-1,j]

# accurate_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) #not linear?
# pixel_coords = np.reshape(accurate_corners, (row,col))

 
# dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
# corner_lines = np.reshape(corners, (row,col))


# def findCorners(img):
#     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#     for i in range(1, 50): #caps out at 50x50 board
#         for j in range(i):
#             row = 2 + i - j
#             col = 3 + j
#             ret, corners = cv2.findChessboardCorners(gray, (row,col), None)
#             if ret:
#                 return corners, (row,col)
#     print "could not find chessboard"
#     return False


# def convertToBoardDists(u, v, corner):




# rotation = calcRotation(corner_lines)





def undistort(cbrow=sys.argv[1], cbcol=sys.argv[2], fileName=sys.argv[3]): # number of black corners in row and column on chessboard picture
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cbrow = int(cbrow)
    cbcol = int(cbcol)

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
    cv2.imshow("img",img)
    cv2.waitKey(500)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,7), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    dist = normalize(np.array([dist[0][i] if i < 2 else 0.0 for i in range(len(dist[0]))]))
    #dist = np.array([-0.13615181, 0.53005398, 0, 0, 0]) # no translation 
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    maps = [] #coordinates calculated by undistorting pixels 

    for corner in corners2:
        a, b = (int(corner[0][0]), int(corner[0][1]))
        maps.append((mapx[b][a], mapy[b][a]))
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

    xy_undistorted = cv2.undistortPoints(corners2, newcameramtx, dist, P=mtx) #coordinates calculated by undistorting points
    xy_undistorted = np.flipud(xy_undistorted)
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('calibresult2.png',dst)
    print('done')
    return xy_undistorted, (cbrow, cbcol)

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

findCenterBoordCoords()
