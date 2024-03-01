#!/usr/bin/python3
from mpu9250 import MPU9250
from madgwick import Madgwick
from orientation import quat2eul, eul2quat
import time
import numpy as np
import math

# ENU:
# X (Pitch): clockwise - / anticlockwise +
# Y (Roll): clockwise - / anticlockwise +
# Z (Yaw): clockwise - / anticlockwise +

# NED:
# X (Roll): clockwise + / anticlockwise -
# Y (Pitch): clockwise + / anticlockwise -
# Z (Yaw): clockwise + / anticlockwise -

if __name__ == "__main__":
    nav_frame = "NED"
    imu = MPU9250(nav_frame=nav_frame)
    filter = Madgwick(9, marg_gain=5)
    hz = 100
    interval = 1/hz
    last = time.time()
    #imu.mpu6500.gyro_calibration(500, 100)
    count = 0
    try:
        while True:
            next = last + interval
            print("dt: ", abs(next - time.time()))
            print("Hz: ", 1/abs(next - time.time()))
            time.sleep(abs(next - time.time()))
            ax, ay, az = imu.get_accel()
            gx, gy, gz = imu.get_gyro()
            mx, my, mz = imu.get_mag()
            roll, pitch, yaw = imu.get_rpy(axis=9)
            temp = imu.get_temp()
            ax, ay, az = round(ax,2), round(ay,2), round(az,2)
            gx, gy, gz = round(gx,2), round(gy,2), round(gz,2)
            mx, my, mz = round(mx,2), round(my,2), round(mz,2)
            roll, pitch, yaw = round(roll,2), round(pitch,2), round(yaw,2)
            temp = round(temp, 2)
            print("")
            print("nav_frame: ", nav_frame)
            print("ax ay az: ", ax, ay, az)
            print("gx gy gz: ", gx, gy, gz)
            print("mx my mz: ", mx, my, mz)
            acc = np.array([[ax],[ay],[az]])
            gyr = np.array([[gx],[gy],[gz]])
            mag = np.array([[mx],[my],[mz]])
            print("roll pitch yaw: ", roll, pitch, yaw)

            if count == 0:
                sin_r2 = np.sin(math.radians(roll)/2)
                sin_p2 = np.sin(math.radians(pitch)/2)
                sin_y2 = np.sin(math.radians(yaw)/2)
                cos_r2 = np.cos(math.radians(roll)/2)
                cos_p2 = np.cos(math.radians(pitch)/2)
                cos_y2 = np.cos(math.radians(yaw)/2)
                w = sin_y2*sin_p2*sin_r2  + cos_y2*cos_p2*cos_r2
                x = -sin_y2*sin_p2*cos_r2 + cos_y2*cos_p2*sin_r2
                y = sin_y2*cos_p2*sin_r2  + cos_y2*sin_p2*cos_r2
                z = sin_y2*cos_p2*cos_r2  - cos_y2*sin_p2*sin_r2
                w, x, y, z = eul2quat(roll, pitch, yaw, seq="zyx")
                filter.init_quat(w, x, y, z)
            print("before madgwick: ", w, x, y, z)
            w, x, y, z = filter.run(acc, gyr, mag, hz)
            roll, pitch, yaw = quat2eul(w, x, y, z, seq="zyx")
            roll, pitch, yaw = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
            roll, pitch, yaw = round(roll,2), round(pitch,2), round(yaw,2)
            print("madgwick roll pitch yaw: ", roll, pitch, yaw)
            print("after madgwick: ", w, x, y, z)
            last = time.time()
            # if count == 2:
            #     exit()
            count += 1
    except KeyboardInterrupt:
        print('interrupted!')

