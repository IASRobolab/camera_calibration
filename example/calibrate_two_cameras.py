'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022
#
# author: Fabio Amadio
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
from camera_utils.camera_init import IntelRealsense
from camera_calibration_lib.cameras_extrinsic_calibration import extrinsic_calibration
from scipy.spatial.transform import Rotation as R
import select
import sys

def stop_loop(stop_entry):
    '''
    Used to quit an infinite loop with a char/string entry
    '''
    rlist = select.select([sys.stdin], [], [], 0.001)[0]
    if rlist and sys.stdin.readline().find(stop_entry) != -1:
        print("Stop Loop detected.")
        return True
    return False

if __name__ == "__main__":
    # 023322061667 : D415
    # 023322062736 : D415
    # 049122251418 : D455
    camera1= IntelRealsense(serial_number='049122251418')
    camera2 = IntelRealsense(serial_number='023322062736')

    chess_size = (5, 4)
    chess_square_size = 40

    print("\033[92mLoop Running. \nTo stop the loop press q and enter.\033[0m")
    while not stop_loop('q'):
        cam1_H_camX = extrinsic_calibration([camera1, camera2], chess_size, chess_square_size, loops = 4, wait_key = False, display_frame = True)
        print("len(cam1_H_camX)", len(cam1_H_camX))
        print(cam1_H_camX)

