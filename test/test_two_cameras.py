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

    camera1 = IntelRealsense(serial_number='023322061667')
    camera2 = IntelRealsense(serial_number='023322062736')
    
    chess_size = (9, 6)
    chess_square_size = 25

    print("\033[92mLoop Running. \nTo stop the loop press q and enter.\033[0m")
    while not stop_loop('q'):
        cam1_H_camX = extrinsic_calibration([camera1, camera2], chess_size, chess_square_size, loops = 4, display_frame = False)
        print(cam1_H_camX)

