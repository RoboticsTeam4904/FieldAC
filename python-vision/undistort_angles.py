import numpy as np
import cv2
import sys
import math




#width = side_length*row
side_length = 0.1
width = 1920
height = 1080
center = np.array([width/2, height/2])

def findCenterTile():
    corner_coords, rowCols = undistort()
    cbrow, cbcol = rowCols
    dists = []
    coords = []
    x, y = center

    #finds corners that are above and to the left of the center
    for coordNum in range(len(corner_coords)):
        if corner_coords[coordNum][0][0] < x and corner_coords[coordNum][0][1] > y:
            dists.append(np.linalg.norm(center - corner_coords[coordNum][0]))
            coords.append(coordNum)

    #finds top left corner
    idx = dists.index(min(dists))
    coord = coords[idx]

    #pixel coords of corners
    top_left_corner = corner_coords[coord]
    top_right_corner = corner_coords[coord + cbrow]
    bottom_left_corner = corner_coords[coord + 1]
    bottom_right_corner = corner_coords[coord + cbrow + 1]

    tile_pixel_coords = np.array([top_left_corner, top_right_corner, bottom_left_corner, bottom_right_corner])
    boardCoords = np.array([[coord // cbrow, coord % cbrow], [(coord + cbrow) // cbrow, (coord + cbrow) % cbrow], [(coord + 1) // cbrow, (coord + 1) % cbrow], [(coord + cbrow + 1) // cbrow, (coord + cbrow + 1) % cbrow]])

    return tile_pixel_coords, boardCoords


def findCenterBoordCoords():
    pxCoords, bdCoords = findCenterTile()
    tl, tr, bl, br = pxCoords #topleft, topright, bottom left, bottom right
    bd_tl, bd_tr, bd_bl, bd_br = bdCoords
    u_1 = np.subtract(br, bl) 
    u_2 = np.subtract(tr, tl)

    #mathematica equation
    x = (1/(2*(-u_1[0][1]*u_2[0][0] + u_1[0][0]*u_2[0][1])))*(-center[1]*u_1[0][0] - tl[0][1]*u_1[0][0] - center[0]*u_1[0][1] + tl[0][0]*u_1[0][1] + bl[0][1]*u_2[0][0] + center[1]*u_2[0][0] - bl[0][0]*u_2[0][1] + center[0]*u_2[0][1] + math.sqrt((center[1]*u_1[0][0] + tl[0][1]*u_1[0][0] + center[0]*u_1[0][1] - tl[0][0]*u_1[0][1] - bl[0][1]*u_2[0][0] - center[1]*u_2[0][0] + bl[0][0]*u_2[0][1] - center[0]*u_2[0][1])**2 - 4*(bl[0][1]*center[0] + bl[0][0]*center[1] - bl[0][1]*tl[0][0] - center[1]*tl[0][0] + bl[0][0]*tl[0][1] - center[0]*tl[0][1])*(-u_1[0][1]*u_2[0][0] + u_1[0][0]*u_2[0][1])))
    p_1 = bl + x*u_1
    L = tl - bl + x*(u_2 - u_1)
    y_1 = (center[0] - p_1[0][0])/L[0][0]
    y_2 = (center[1] - p_1[0][1])/L[0][1]
 
    if y_1 != y_2:
        diff = y_1 - y_2
        print("diff: " , str(diff))
        if diff > 50:
            print("Rip calculations")
    print(y_1, y_2)

    if y_1 > 0 and y_1 < 1:
        y = y_1
    elif y_2 > 0 and y_2 < 1:
        y = y_2
    else:
        y = 0
    center_board_coords = np.array([bd_bl[0] + x, bd_bl[1] + y]) 
    print center_board_coords
    return center_board_coords

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
    print ret

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

center_board_coords = findCenterBoordCoords()

# corners, row, col = findCorners(img)
row, col = int(sys.argv[1]), int(sys.argv[2])
corner_grid = np.zeros((row, col))#np.reshape(corners, (row,col))

# # -----------Calculate x and y angles, an array called thetas -----------
# Uses the distances to the corners of the board and the center pixel's coordinates on the board

dist_to_top_left = 10
dist_to_top_right = 10
dist_to_bottom_left = 10
dist_to_bottom_right = 10

# make sure you know which way the coordinates go (indeces order vs top right and such)
true_dists = np.zeros_like(corner_grid) #array of floats
true_dists[0,0] = dist_to_top_left
true_dists[0,col-1] = dist_to_top_right
true_dists[row-1,0] = dist_to_bottom_left
true_dists[row-1,col-1] = dist_to_bottom_right

board_width = side_length*row
board_height = side_length*col

def calcAngle(a, b, c):
    return math.acos((a**2 + b**2 - c**2)/(2*a*b))

def calcDist(theta, a, b):
    return math.sqrt(a**2 + b**2 - 2*a*b*math.cos(theta))

angles_right = np.zeros((row))
angles_down = np.zeros((col))
thetas = np.zeros((row,col,2))

angles_down[0] = calcAngle(true_dists[0,0], board_height, true_dists[row-1,0])
angles_down[col-1] = calcAngle(true_dists[0,col-1], board_height, true_dists[row-1,col-1])
for i in range(1,row-1): # Don't include corners
    true_dists[i,0] = calcDist(angles_down[0], side_length*i, true_dists[0,0]) # need new angles for inner grid dists
    true_dists[i,col-1] = calcDist(angles_down[col-1], side_length*i, true_dists[0,col-1])
    angles_right[i] = calcAngle(true_dists[i,0], board_width, true_dists[i,col-1])

angles_right[0] = calcAngle(true_dists[0,0], board_width, true_dists[0,col-1]) # Only needed to get angles_right, could otherwise just iterate using angle downs
angles_right[row-1] = calcAngle(true_dists[row-1,0], board_width, true_dists[row-1,col-1])
for j in range(1,col-1):
    true_dists[0,j] = calcDist(angles_right[0], side_length*j, true_dists[0,0])
    true_dists[row-1,j] = calcDist(angles_right[row-1], side_length*j, true_dists[row-1,0])
    angles_down[j] = calcAngle(true_dists[0,j], board_height, true_dists[row-1,j])

for i in range(row):
    for j in range(col):
        dist = calcDist(angles_down[j], side_length*i, true_dists[0,j])
        if 0 < i < row-1 and 0 < j < col-1: # doesn't actually matter, will just reset to same value
            true_dists[i,j] = dist
        # x_axis_analog = (center_board_coords[0], j)
        # y_axis_analog = (i, center_board_coords[1])
        x_analog_dist = calcDist(angles_down[j], side_length*center_board_coords[0], true_dists[0,j])
        y_analog_dist = calcDist(angles_right[i], side_length*center_board_coords[1], true_dists[i,0])
        y_theta = calcAngle(dist, x_analog_dist, abs(i-center_board_coords[0]))
        x_theta = calcAngle(dist, y_analog_dist, abs(j-center_board_coords[1]))
        thetas[i,j,0] = x_theta
        thetas[i,j,1] = y_theta
