#!/usr/bin/python3
import yaml
from mpu9250_driver.mpu9250 import MPU9250
from utils.orientation import quat2eul, eul2quat
import time
import numpy as np
import math

if __name__ == '__main__':
    """
    Do calibration for gyroscope and accelerometer
    """
    nav_frame = "NED" # ENU/NED
    axis = 9
    hz = 100
    interval = 1/hz
    acc_time = 1
    gyro_time = 1
    mag_time = 10
    calibration = True
    imu = MPU9250(nav_frame, axis, hz, calibration)
    imu.initialization()
    gyro_scale, gyro_bias, gyro_misalignment = imu.mpu6500.gyro_calibration(gyro_time)
    accel_scale, accel_bias, accel_misalignment = imu.mpu6500.accel_calibration(acc_time)
    mag_scale, mag_bias, mag_misalignment, mag_strength = imu.ak8963.mag_calibration(mag_time)
    bias = np.vstack((np.vstack((gyro_bias,accel_bias)),mag_bias))
    scale = np.vstack((np.vstack((gyro_scale,accel_scale)),mag_scale))
    misalignment = np.vstack((np.vstack((gyro_misalignment,accel_misalignment)),mag_misalignment))

    # Refresh new config to yaml file
    config = yaml.load(open("../cfg/config.yaml", "r"), Loader=yaml.FullLoader)
    bias_parameter = ["gx_bias","gy_bias","gz_bias","ax_bias","ay_bias","az_bias","mx_bias","my_bias","mz_bias"]
    scale_parameter = ["gx_scale","gy_scale","gz_scale","ax_scale","ay_scale","az_scale","mx_scale","my_scale","mz_scale"]
    
    bias_scale = {}
    for i, element in enumerate(bias_parameter):
        bias_scale[element] = float(bias[i][0])
    for i, element in enumerate(scale_parameter):
        bias_scale[element] = float(scale[i][0])
    config[nav_frame] = bias_scale
    with open("config.yaml", 'w') as file:
        file.write(yaml.dump(config))
    print("accel bias: ")
    print(accel_bias)
    print("gyro bias: ")
    print(gyro_bias)
    print("mag bias: ")
    print(mag_bias)
    print("accel scale: ")
    print(accel_scale)
    print("gyro scale: ")
    print(gyro_scale)
    print("mag scale: ")
    print(mag_scale)