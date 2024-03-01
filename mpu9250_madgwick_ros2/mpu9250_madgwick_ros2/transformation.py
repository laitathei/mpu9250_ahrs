#!/usr/bin/python3
import numpy as np
from .orientation import *

# Accelerometer measure 3 axes acceleration of its Body frame
# DCM sequence aims to convert from Body to Navigation frame
# Gravity vector is in Navigation frame
# The seq in this program refers to DCM sequence
# [fx]                [0]
# [fy] = [DCM (ZXY)].T[0]
# [fz]                [g]
# body             Navigation

def acc2eul(ax, ay, az, nav="ENU"):
    """
    Convert acceleration vector with gravity to Euler angle in ENU or NED frame
    Input: ax, ay, az
    Output: roll, pitch, yaw (radians)

    .. Reference
    .. [1] Page.163, Fundamentals of Inertial Navigation, Satellite-based Positioning and their Intergration, Springer, 2013 
    .. [2] https://www.nxp.com/docs/en/application-note/AN5017.pdf (Gimbal Lock explaination)
    """
    roll = 0
    pitch = 0
    yaw = 0
    acc = np.array([[ax,ay,az]])
    if np.linalg.norm(acc) > 0:
        acc_norm = np.linalg.norm(acc)
        ax = ax/acc_norm
        ay = ay/acc_norm
        az = az/acc_norm
        if nav=="ENU": # ZXY (yaw - pitch - roll)
            # pitch limited between +- 90 degrees as Gimbal Lock problem (Singularity)
            # [ax (E)] = [-sin_y*sin_p*sin_r+cos_y*cos_p   cos_y*sin_p*sin_r+sin_y*cos_p    -sin_p*cos_r][0]
            # [ay (N)] = [-sin_y*cos_r                     cos_y*cos_r                      sin_r       ][0]
            # [az (U)] = [sin_y*cos_p*sin_r+cos_y*sin_p    -cos_y*cos_p*sin_r+sin_y*sin_p   cos_p*cos_r ][g]
            roll = np.arctan2(-ax, az)
            pitch = np.arctan2(ay, np.sqrt(ax**2 + az**2)) # or np.arcsin(ay)
            yaw = 0.0
        elif nav=="NED": # ZYX (yaw - pitch - roll)
            # pitch limited between +- 90 degrees as Gimbal Lock problem (Singularity)
            # [ax (N)] = [cos_y*cos_p                     sin_y*cos_p                     -sin_p     ][0]
            # [ay (E)] = [cos_y*sin_p*sin_r-sin_y*cos_r   sin_y*sin_p*sin_r+cos_y*cos_r   cos_p*sin_r][0]
            # [az (D)] = [cos_y*sin_p*cos_r+sin_y*sin_r   sin_y*sin_p*cos_r-cos_y*sin_r   cos_p*cos_r][g]
            roll = np.arctan2(ay, az)
            pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2)) # or np.arcsin(-ax)
            yaw = 0.0
        else:
            raise ValueError("Navigation frame should be either ENU or NED")
    return roll, pitch, yaw

def acc2quat(ax, ay, az, nav="ENU"):
    """
    Convert acceleration vector with gravity to Quaternion in ENU or NED frame
    Input: ax, ay, az
    Output: w, x, y, z

    .. Reference
    .. [1] Page.162, Fundamentals of Inertial Navigation, Satellite-based Positioning and their Intergration, Springer, 2013 
    .. [2] https://www.nxp.com/docs/en/application-note/AN3461.pdf
    .. [3] https://ntrs.nasa.gov/api/citations/19770019231/downloads/19770019231.pdf
    .. [4] https://zhuanlan.zhihu.com/p/45404840
    """
    roll, pitch, yaw = acc2eul(ax, ay, az, nav)
    if nav=="ENU": # ZXY (yaw - pitch - roll)
        # w = -np.sin(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)
        # x = -np.sin(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)
        # y = np.sin(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)
        # z = np.sin(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)+np.cos(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)
        w, x, y, z = eul2quat(roll, pitch, yaw, seq="zxy")
    elif nav=="NED": # ZYX (yaw - pitch - roll)
        # w = np.sin(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)
        # x = -np.sin(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)
        # y = np.sin(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)
        # z = np.sin(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)-np.cos(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)
        w, x, y, z = eul2quat(roll, pitch, yaw, seq="zyx")
    else:
        raise ValueError("Navigation frame should be either ENU or NED")
    return w, x, y, z

def accmag2eul(ax, ay, az, mx, my, mz, nav="ENU"):
    """
    Convert acceleration vector with gravity and magnetometer value to Euler angle in ENU or NED frame
    Input: ax, ay, az, mx, my, mz
    Output: roll, pitch, yaw (radians)

    .. Reference
    .. [1] https://www.st.com/resource/en/design_tip/dm00269987-computing-tilt-measurement-and-tilt-compensated-e-compass-stmicroelectronics.pdf
    .. [2] https://blog.csdn.net/qq_38313901/article/details/126464285
    """
    roll, pitch, yaw = acc2eul(ax, ay, az, nav) # accelerometer provide roll and pitch angle
    mag = np.array([[mx],[my],[mz]])
    if np.linalg.norm(mag) > 0:
        mag_norm = np.linalg.norm(mag)
        mx = mx/mag_norm
        my = my/mag_norm
        mz = mz/mag_norm
        if nav == "ENU":
            # bx = mx*(-sin_y*sin_r*sin_p+cos_y*cos_p) + my*(-sin_y*cos_r) + mz*(sin_y*sin_p*cos_r+cos_y*sin_r)
            # by = mx*(cos_y*sin_p*sin_r+sin_y*cos_r)  + my*(cos_y*cos_p)  + mz*(-cos_p*sin_p*cos_r+sin_y*sin_r)
            # bz = mx*(-cos_p*sin_r)                   + my*(sin_p)        + mz*(cos_p*cos_r)
            DCM = eul2dcm(roll, pitch, yaw, seq="zxy")
            result = DCM @ mag
            X = result.tolist()[0][0]
            Y = result.tolist()[1][0]
            Z = result.tolist()[2][0]
            yaw = np.arctan2(X, Y)
        elif nav == "NED":
            # bx = mx*(cos_y*cos_p) + my*(cos_y*sin_p*sin_r-sin_y*cos_r)  + mz*(cos_y*sin_p*cos_r+sin_y*sin_r)
            # by = mx*(sin_y*cos_p) + my*(sin_y*sin_p*sin_r+cos_y*cos_r)  + mz*(sin_y*sin_p*cos_r-cos_y*sin_r)
            # bz = mx*(-sin_p)      + my*(cos_p*sin_r)                    + mz*(cos_p*cos_r)
            DCM = eul2dcm(roll, pitch, yaw, seq="zyx")
            result = DCM @ mag
            X = result.tolist()[0][0]
            Y = result.tolist()[1][0]
            Z = result.tolist()[2][0]
            yaw = np.arctan2(Y, X)
        else:
            raise ValueError("Navigation frame should be either ENU or NED")
    return roll, pitch, yaw

def accmag2quat(ax, ay, az, mx, my, mz, nav="ENU"):
    """
    Convert acceleration vector with gravity and magnetometer value to Quaternion in ENU or NED frame
    Input: ax, ay, az, mx, my, mz
    Output: w, x, y, z
    """
    roll, pitch, yaw = accmag2eul(ax, ay, az, mx, my, mz, nav)
    if nav=="ENU": # ZXY (yaw - pitch - roll)
        # w = -np.sin(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)
        # x = -np.sin(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)
        # y = np.sin(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)
        # z = np.sin(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)+np.cos(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)
        w, x, y, z = eul2quat(roll, pitch, yaw, seq="zxy")
    elif nav=="NED": # ZYX (yaw - pitch - roll)
        # w = np.sin(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)
        # x = -np.sin(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)+np.cos(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)
        # y = np.sin(yaw/2)*np.cos(pitch/2)*np.sin(roll/2)+np.cos(yaw/2)*np.sin(pitch/2)*np.cos(roll/2)
        # z = np.sin(yaw/2)*np.cos(pitch/2)*np.cos(roll/2)-np.cos(yaw/2)*np.sin(pitch/2)*np.sin(roll/2)
        w, x, y, z = eul2quat(roll, pitch, yaw, seq="zyx")
    else:
        raise ValueError("Navigation frame should be either ENU or NED")
    return w, x, y, z

def ENU2NED(E, N, U):
    """
    Convert data from NED frame to ENU frame
    Input: E, N, U
    Output: N, E, D
    """
    ENU = np.array([[E],
                    [N],
                    [U]])
    matrix = np.array([[0,1,0],
                       [1,0,0],
                       [0,0,-1]])
    NED = matrix @ ENU
    N = NED[0][0]
    E = NED[1][0]
    D = NED[2][0]
    return N, E, D

def NED2ENU(N, E, D):
    """
    Convert data from ENU frame to NED frame
    Input: N, E, D
    Output: E, N, U
    """
    NED = np.array([[N],
                    [E],
                    [D]])
    matrix = np.array([[0,1,0],
                       [1,0,0],
                       [0,0,-1]])
    ENU = matrix @ NED
    E = ENU[0][0]
    N = ENU[1][0]
    U = ENU[2][0]
    return E, N, U
