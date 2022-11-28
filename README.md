# camera_calibration

This package contains functions for camera intrinsic and extrinsic calibration.
___

### Dependencies
___
The library contains python modules which are dependent on the following 3rd-party libraries:
```
opencv-python, numpy, matplotlib, setuptools
```
Additionally, it is necessary to install our [camera_utils](https://github.com/IASRobolab/camera_utils).

### Installation
___
To install the camera_calibration package on your system, clone the GitHub repository in a folder of your choice, open the cloned repository path in a terminal and run the following command

```
python3 -m pip install .
```

Instead if you want to install the package in "editable" or "develop" mode (to prevent the uninstall/install of the
package at every pkg modification) you have can run the following command:

```
python3 -m pip install -e .
```

## Usage
___

- __camera_calibration__: this function perform the intrinsic and extrinsic calibration of a camera
- __chessboard_pose_estimation__: this function returns the estimate of chessboard position and orientation w.r.t. the camera get_frame
- __extrinsic_calibration__: this function perform the extrinsic calibration between a list of cameras

Inside the __example__ folder you can find a simple script to perform extrinsic calibration of two IntelRealsense cameras: __calibrate_two_cameras.py__.

## Authors
___
The package is provided by:

- [Federico Rollo](https://github.com/FedericoRollo) [Mantainer]
- [Fabio Amadio](https://github.com/fabio-amadio)
