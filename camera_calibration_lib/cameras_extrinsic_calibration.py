'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022
#
# author: Federico Rollo, Fabio Amadio
# mail: rollo.f96@gmail.com
#
# Institute: Leonardo Labs (Leonardo S.p.a - Istituto Italiano di tecnologia)
#
# This file is part of camera_calibration. <https://github.com/IASRobolab/camera_calibration>
#
# camera_calibration is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# camera_calibration is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License. If not, see http://www.gnu.org/licenses/
---------------------------------------------------------------------------------------------------------------------------------'''
import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

from camera_calibration_lib.chessboard_pose_estimation import chessboard_pose_estimation

def extrinsic_calibration(cameras, chess_size, chess_square_size, loops = 1, wait_key = False, display_frame = False):
    '''
    @brief: this function perform the extrinsic calibration between a list of cameras.

    @param cameras [mandatory]: [list] is a list of Camera type. see camera_utils.camera_init for more info.

    @param chessboard_size [mandatory]: [tuple(int)] is the number of chessboard corners in tuple form i.e. (length, width).

    @param chess_square_size [mandatory]: [int] is the length of a chessboard square in mm.

    @param loops: [int] number of samples for computing average poses

    @param display_frame: [boolean] flag to allow display of image and frame

    @param wait_key: [boolean] flag to set the wait-for-user-input modality

    @return: cam1_H_camX [list(np.array(4x4))] is a list of homogeneous transformation of all the cameras in the cameras list with respect to the first camera of the list. 
    '''
    assert(len(cameras) >= 2 and "You must pass at least two cameras in the cameras list")

    # discard first frames because they are not good
    for camera in cameras:
        for i in range(10):
            camera.get_rgb()
    
    cam1_H_camX_samples = []
    for k in tqdm(range(loops)):
        homogeneous_matrices = [] # store chessboard poses from each camera
        if wait_key:
                input("Position the chessboard and press any key")
        for camera in cameras:
            rot_mat, trasl_vec = chessboard_pose_estimation(camera, chess_size, chess_square_size, display_frame)
            hom_mat = np.eye(4)
            hom_mat[:3, :3] = rot_mat
            hom_mat[:3, 3:] = trasl_vec
            homogeneous_matrices.append(hom_mat)

        cam1_H_camX = [] # is a list of homogeneous transforamtion between cam1 and all the other cam in cameras list
        H_ref = homogeneous_matrices[0]
        for hom_mat in homogeneous_matrices:
            H_inv = np.linalg.inv(hom_mat)
            cam1_H_currentCam = np.matmul(H_ref, H_inv)
            cam1_H_camX.append(cam1_H_currentCam)
        
        cam1_H_camX_samples.append(cam1_H_camX[1:])

    cam1_H_camX_samples = np.array(cam1_H_camX_samples)
    cam1_H_camX_mean = np.mean(cam1_H_camX_samples, 0)
    new_cam1_H_camX = []
    for i in range(cam1_H_camX_mean.shape[0]):
        new_cam1_H_camX.append(cam1_H_camX_mean[i])
    return new_cam1_H_camX