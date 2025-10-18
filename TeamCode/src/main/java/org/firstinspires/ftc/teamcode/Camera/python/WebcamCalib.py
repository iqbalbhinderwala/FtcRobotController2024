# From: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

# ------------------------------------------------------------------------------------------------
# pip install -r requirements.txt
# ------------------------------------------------------------------------------------------------

import numpy as np
import cv2 as cv
import glob

# ------------------------------------------------------------------------------------------------
# Logitech HD Webcam C270
# ------------------------------------------------------------------------------------------------

# NAME = "Logitech HD Webcam C270"
# VID  = "Logitech"
# PID  = "0x0825"
# IMAGE_DIR = 'C:/ftc/LogitechC270/*.png'

# ------------------------------------------------------------------------------------------------
# Logitech Webcam C930e
# ------------------------------------------------------------------------------------------------

NAME = "Logitech Webcam C930e"
VID  = "Logitech"
PID  = "0x0843"
IMAGE_DIR = 'C:/ftc/LogitechC930e_640x480/*.png'
# IMAGE_DIR = 'C:/ftc/LogitechC930e_800x448/*.png'
# IMAGE_DIR = 'C:/ftc/LogitechC930e_800x600/*.png'
# IMAGE_DIR = 'C:/ftc/LogitechC930e_848x480/*.png'
# IMAGE_DIR = 'C:/ftc/LogitechC930e_960x540/*.png'
# IMAGE_DIR = 'C:/ftc/LogitechC930e_1280x720/*.png'

# ------------------------------------------------------------------------------------------------
# Set INSPECT_FRAMES to True to inspect each frame, False for quick run 
# If any detected corner is not good in a given image, delete that image from the set and rerun
# ------------------------------------------------------------------------------------------------
INSPECT_FRAMES = True 
# INSPECT_FRAMES = False

# ------------------------------------------------------------------------------------------------
# Grid size (number of corners)
# ------------------------------------------------------------------------------------------------
NC = 8
NR = 6

# ------------------------------------------------------------------------------------------------

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((NR*NC,3), np.float32)
objp[:,:2] = np.mgrid[0:NC,0:NR].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
images = glob.glob(IMAGE_DIR)
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (NC,NR), None)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (NC,NR), corners2, ret)

        if INSPECT_FRAMES:
            cv.imshow(fname, img)
            cv.waitKey()
        else:
            cv.imshow("img", img)
            cv.waitKey(100)
 
cv.destroyAllWindows()

#     distortionCoefficients (an 8-element float array): distortion coefficients in the following form
#         (r:radial, t:tangential): [r0, r1, t0, t1, r2, r3, r4, r5]
#         see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

# Calibration
flags = None
flags = cv.CALIB_ZERO_TANGENT_DIST 
# flags = cv.CALIB_RATIONAL_MODEL  
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None, flags=flags
)

# Calculate reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print('=======================================================================================')
print(NAME)
print(IMAGE_DIR)
print('=======================================================================================')

print("\nCamera matrix :")
print(mtx)
print("\nDistortion Coefficients:")
print(dist.flatten())
print('=======================================================================================')
print("Total error: {:.4f}".format(mean_error/len(objpoints)) )
if mean_error/len(objpoints) > 0.04:
    print("WARNING: High reprojection error. Consider redoing calibration with better images.")
    # Set INSPECT_FRAMES = True and delete bad images!
print('=======================================================================================')

dist8 = np.zeros((8, 1))
flat_dist = dist.flatten()

# Fill dist8 with available values, pad remainder with zeros
for i in range(min(8, len(flat_dist))):
    dist8[i, 0] = flat_dist[i]


print(' ')
print('Add the following to TeamCode/src/main/res/xml/teamwebcamcalibrations.xml:')
print(' ')

print('<!-- ======================================================================================= -->')
print('')
print('<!-- {} Calibration - By ShortCircuit 2025 - Via OpenCV -->'.format(NAME))
print('<Camera vid="{}" pid="{}">'.format(VID, PID))
print('    <Calibration')
print('        size="{} {}"'.format(gray.shape[1], gray.shape[0]))
print('        focalLength="{:.3f}f, {:.3f}f"'.format(mtx[0,0], mtx[1,1]))
print('        principalPoint="{:.3f}f, {:.3f}f"'.format(mtx[0,2], mtx[1,2]))
print('        distortionCoefficients="{}"'.format(', '.join(['{:.6g}'.format(coef) for coef in dist8.flatten()])))
print('        />')
print('</Camera>')
print('')
print('<!-- ======================================================================================= -->')

print(' ')
print('Alternatively, add the following to AprilTagProcessor.Builder(). (see ConceptAprilTag.java):')
print('#'*60)
print("     .setLensIntrinsics({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2]))
print('#'*60)
