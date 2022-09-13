'''
This script is used to find intrinsic, extrinsic and distortion parameters from a camera
'''
import cv2
import numpy as np


def camera_calibration(camera, chessboard_size, chess_square_size, display_chess_pattern=False):
    '''
    @brief: this function perform the intrinsic and extrinsic calibration of a camera.

    @param camera [mandatory]: [Camera] is Camera type object. see camera_utils.camera_init for more info.

    @param chessboard_size [mandatory]: [tuple(int)] is the number of chessboard corners in tuple form i.e. (length, width).

    @param chess_square_size [mandatory]: [int] is the length of a chessboard square in mm.

    @param display_chess_pattern: [bool] if you want to plot the image with the chessboard pattern.

    @return: mtx intrinsic parameters matrix, dist distortion parameters of the camera and rmat and tvecs extrinsic parameters (rotation and translation).
    '''
    
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # scaling factor from mm to cm
    mm2m = 1000 
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2) * chess_square_size / mm2m

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    current_acquisition = 0
    acquisition_number = 300
    while current_acquisition < acquisition_number:
        img = camera.get_rgb()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            if display_chess_pattern:
                # Draw and display the corners
                cv2.drawChessboardCorners(img, (9,6), corners2, ret)
                cv2.namedWindow('img', cv2.WINDOW_NORMAL)
                cv2.imshow('img', img)
                cv2.waitKey(1)
            current_acquisition += 1

    print("aquisition finished, starting camera calibration. It can take some minutes.")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    ## Example of image undistortion ##
    # img = camera.get_rgb()
    # h,  w = img.shape[:2]
    # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # # undistort
    # dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # # crop the image
    # x, y, w, h = roi
    # dst = dst[y:y+h, x:x+w]
    ## 

    ## Error evaluation of obtained params values ##
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )
    ##

    return mtx, dist
