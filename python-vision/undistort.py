import numpy as np
import cv2
import sys

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
    undist_pts = np.flipud(xy_undistorted)
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('calibresult2.png',dst)
    print('done')
    print(undist_pts)
    return undist_pts

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

undistort()
