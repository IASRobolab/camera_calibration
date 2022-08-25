from __future__ import barry_as_FLUFL
import pdb

from camera_utils.camera_init import IntelRealsense
import cv2
import numpy as np

def chessboard_pose_estimation(camera, chessboard_size, chess_square_size = 10, display_frame = False):
    '''
    @brief this function returns the estimate of chessboard position and orientation wrt the camera.

    @param camera: [mandatory] is an Object of Camera type (e.g. IntelRealsense, Zed). 
    see camera_utils.camera_init for more info.

    @param chessboard_size: [mandatory] is the number of chessboard corners in tuple form i.e. (length, width)

    @param chess_square_size: is the length  of a chessboard square in mm.

    @param display_frame: boolean value to display in 2D plane the frame d

    @return rot_matrix is a 3x3 rotation matrix, trans_vec is a 3x1 translation vector. 
    They represent the pose of the camera wrt the chessboard top-left angle
    '''
    # get intrinsics params and distortion params
    intr = camera.get_intrinsics()
    mtx = np.array([[intr['fx'], 0, intr['px']], [0, intr['fy'], intr['py']], [0, 0, 1]])
    dist = np.array([[0., 0., 0., 0., 0.]])

    mm2cm = 10 # scaling factor from mm to cm
    # initialize criteria, object points and axis for solving PnP problem
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2) * chess_square_size / mm2cm

    pose_found = False
    while not pose_found:

        img = camera.get_rgb()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        chessboard_found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if chessboard_found == True:
            # Refine previously found corners
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Find the rotation and translation vectors.
            pose_found, rot_vec, trans_vec = cv2.solvePnP(objp, corners2, mtx, dist)

            # Compute rotation matrics from rotation vector
            rot_matrix = cv2.Rodrigues(rot_vec)[0]

            ## Print reference frame on image plane ##
            if display_frame:
                # project 3D points to image plane
                axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
                imgpts, jac = cv2.projectPoints(axis, rot_vec, trans_vec, mtx, dist)
                corner = tuple(corners2[0].ravel().astype(int))

                img = cv2.line(img, corner, tuple(imgpts[0].ravel().astype(int)), (255,0,0), 5)
                img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype(int)), (0,255,0), 5)
                img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype(int)), (0,0,255), 5)
                cv2.imshow('img', img)

                if cv2.waitKey(0) == ord('q'):
                    print("Calibration stopped.")
                    exit(0)

        cv2.destroyAllWindows()
        return rot_matrix, trans_vec


