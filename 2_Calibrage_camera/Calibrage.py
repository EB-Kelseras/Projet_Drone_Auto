#%%
import numpy as np
import cv2 as cv
import glob
import os

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*4,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:4].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Obtenir le chemin du répertoire où se trouve le script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Construire le chemin vers le dossier d'images
images_path = os.path.join(script_dir, 'DataB_Images', '*.png')

images = glob.glob(images_path)
gray = None
 #%%
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imshow('img', gray)
    cv.waitKey(1000)
    
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (6,4), None)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (6,4), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(1000)
 
cv.destroyAllWindows()
#%%
if gray is not None:
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
else:
    print("No image detected")

countImg = 1
for fname in images:
    
    img = cv.imread(fname)
    h, w = img.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # undistort
    mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
    dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    image_path = os.path.join(script_dir, 'calibresult_{}.png'.format(countImg))
    cv.imwrite(image_path, dst)
    countImg += 1

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
 
print( "total error: {}".format(mean_error/len(objpoints)) )
# %%
