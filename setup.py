from setuptools import setup

package_name = 'camera_calibration_lib'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'matplotlib'],
    zip_safe=True,
    maintainer='Federico Rollo',
    maintainer_email='rollo.f96@gmail.com',
    description='This package contains function for camera intrinsic and extrinsic calibration.',
    license='GNU GENERAL PUBLIC LICENSE v3',
    entry_points={},
)
