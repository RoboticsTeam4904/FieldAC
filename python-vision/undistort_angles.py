import numpy as np
import cv2
import sys
import math


def findPixelTile(coord, corner_coords, row, col):
    dists = []
    coords = []
    x, y = coord

    # corner_coords = np.reshape(corner_coords, (row, col))

    #finds corners that are above and to the left of the center
    for coordNum in range(len(corner_coords)):
        if corner_coords[coordNum][0][0] < x and corner_coords[coordNum][0][1] > y:
            dists.append(np.linalg.norm(coord - corner_coords[coordNum][0]))
            coords.append(coordNum)

    #finds top left corner
    idx = dists.index(min(dists))
    coord = coords[idx]

    #pixel coords of corners
    top_left_corner = corner_coords[coord]
    top_right_corner = corner_coords[coord + row]
    bottom_left_corner = corner_coords[coord + 1]
    bottom_right_corner = corner_coords[coord + row + 1]

    tile_pixel_coords = np.array([top_left_corner, top_right_corner, bottom_left_corner, bottom_right_corner])
    boardCoords = np.array([[coord // row, coord % row], [(coord + row) // row, (coord + row) % row], [(coord + 1) // row, (coord + 1) % row], [(coord + row + 1) // row, (coord + row + 1) % row]])

    return tile_pixel_coords, boardCoords


def findPixelBoardCoords(coord, pixel_coords, board_coords):

    tl, tr, bl, br = pixel_coords #topleft, topright, bottom left, bottom right
    bd_tl, bd_tr, bd_bl, bd_br = board_coords
    u_1 = np.subtract(br, bl) 
    u_2 = np.subtract(tr, tl)

    #mathematica equation
    x = (1/(2*(-u_1[0][1]*u_2[0][0] + u_1[0][0]*u_2[0][1])))*(-coord[1]*u_1[0][0] - tl[0][1]*u_1[0][0] - coord[0]*u_1[0][1] + tl[0][0]*u_1[0][1] + bl[0][1]*u_2[0][0] + coord[1]*u_2[0][0] - bl[0][0]*u_2[0][1] + coord[0]*u_2[0][1] + math.sqrt((coord[1]*u_1[0][0] + tl[0][1]*u_1[0][0] + coord[0]*u_1[0][1] - tl[0][0]*u_1[0][1] - bl[0][1]*u_2[0][0] - coord[1]*u_2[0][0] + bl[0][0]*u_2[0][1] - coord[0]*u_2[0][1])**2 - 4*(bl[0][1]*coord[0] + bl[0][0]*coord[1] - bl[0][1]*tl[0][0] - coord[1]*tl[0][0] + bl[0][0]*tl[0][1] - coord[0]*tl[0][1])*(-u_1[0][1]*u_2[0][0] + u_1[0][0]*u_2[0][1])))
    p_1 = bl + x*u_1
    L = tl - bl + x*(u_2 - u_1)
    y_1 = (coord[0] - p_1[0][0])/L[0][0]
    y_2 = (coord[1] - p_1[0][1])/L[0][1]
 
    if y_1 != y_2:
        diff = y_1 - y_2
        print("diff: " , str(diff))
        if diff > 50:
            print("Rip calculations")
    print(y_1, y_2)

    if 0 <= y_1 <= 1:
        y = y_1
    elif 0 <= y_2 <= 1:
        y = y_2
    else:
        print "y1 and y2", y_1, y_2
        raise ValueError("Could not find a correct center y coord")
        quit()
    center_board_coords = np.array([bd_bl[0] + x, bd_bl[1] + y]) 
    print center_board_coords
    return center_board_coords


def findBoard(img, row, col, display=False): # number of black corners in row and column on chessboard picture
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (row,col), None)

    # If found, add object points, image points (after refining them)
    if not ret:
        raise Exception("COULD NOT FIND BOARD")
        quit()

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    sub_pix_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

    if display:
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,7), sub_pix_corners,ret)
        cv2.imshow('img', img)
        cv2.waitKey(1000)

    return sub_pix_corners


def undistort(corners, row, col, display=False): # number of black corners in row and column on chessboard picture

    # Arrays to store object points and image points from all the images.
    # 3d point in real world space like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((row*col,3), np.float32)
    objp[:,:2] = np.mgrid[0:col,0:row].T.reshape(-1,2)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([objp], [corners], (img_width,img_height), None, None)
    dist = normalize(np.array([dist[0][i] if i < 2 else 0.0 for i in range(len(dist[0]))]))
    #dist = np.array([-0.13615181, 0.53005398, 0, 0, 0]) # no translation 
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(img_width,img_height),1,(img_width,img_height))

    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(img_width,img_height),5)
    maps = [] #coordinates calculated by undistorting pixels 

    for corner in corners:
        a, b = (int(corner[0][0]), int(corner[0][1]))
        maps.append((mapx[b][a], mapy[b][a]))

    xy_undistorted = cv2.undistortPoints(corners, newcameramtx, dist, P=mtx) #coordinates calculated by undistorting points
    xy_undistorted = np.flipud(xy_undistorted)

    if display:
        img_undistorted = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        cv2.imshow('calibresult2.png', img_undistorted)

    return xy_undistorted, (row, col)

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

# # -----------Calculate x and y angles, an array called thetas -----------
# Uses the distances to the corners of the board and the center pixel's coordinates on the board
def calculateXYAngles(dists, center_board_coords, row, col, side_length):
    dist_to_top_left, dist_to_top_right, dist_to_bottom_left, dist_to_bottom_right = dists
    # make sure you know which way the coordinates go (indices order vs top right and such)
    true_dists = np.zeros((row, col)) #array of floats
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
            x_analog_dist = calcDist(angles_down[j], side_length*center_board_coords[0], true_dists[0,j])
            y_analog_dist = calcDist(angles_right[i], side_length*center_board_coords[1], true_dists[i,0])
            y_theta = calcAngle(dist, x_analog_dist, abs(i-center_board_coords[0]))
            x_theta = calcAngle(dist, y_analog_dist, abs(j-center_board_coords[1]))
            thetas[i,j,0] = x_theta
            thetas[i,j,1] = y_theta

    return thetas

side_length = 0.2
dist_to_top_left = 10
dist_to_top_right = 10
dist_to_bottom_left = 10
dist_to_bottom_right = 10
dists = (dist_to_top_left, dist_to_top_right, dist_to_bottom_left, dist_to_bottom_right)

row, col, file_name = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
img = cv2.imread(file_name)
img_height, img_width = img.shape[:2]
center = np.array([img_width/2, img_height/2])

distorted_corners = findBoard(img, col, row)
corner_coords, rowCols = undistort(distorted_corners, row, col)
pixel_coords, board_coords = findPixelTile(center, corner_coords, row, col)
center_board_coords = findCenterPixelCoords(center, pixel_coords, board_coords)
thetas = calculateXYAngles(dists, center_board_coords, side_length, row, col)


