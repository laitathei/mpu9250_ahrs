#!/usr/bin/python3
import numpy as np
from .orientation import eul2quat

# Accelerometer measure 3 axes acceleration of its Body frame
# DCM sequence aims to convert from Body to Navigation frame
# Gravity vector is in Navigation frame
# The seq in this program refers to DCM sequence
# [ax]                [0]
# [ay] = [DCM (ZXY)].T[0]
# [az]                [g]
# body             Navigation

def acc2eul(ax, ay, az, nav="ENU"):
    """
    Convert acceleration vector with gravity to Euler angle in ENU or NED frame [1]_ [2]_

    :param float ax: x axis accelerometer
    :param float ay: y axis accelerometer
    :param float az: z axis accelerometer
    :param str nav: navigation frame
    :returns: 
        - roll (float) - x-axis Euler angle in radians
        - pitch (float) - y-axis Euler angle in radians
        - yaw (float) - z-axis Euler angle in radians

    .. Reference
    .. [1] Page.163, Fundamentals of Inertial Navigation, Satellite-based Positioning and their Intergration, Springer, 2013 
    .. [2] 'Gimbal Lock explaination <https://www.nxp.com/docs/en/application-note/AN5017.pdf>'
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
        if nav=="ENU": # ZXY (yaw - roll - pitch)
            # roll limited between +- 90 degrees as Gimbal Lock problem (Singularity)
            # [ax (E)] = [-sin_y*sin_p*sin_r+cos_y*cos_p   cos_y*sin_p*sin_r+sin_y*cos_p    -sin_p*cos_r][0]
            # [ay (N)] = [-sin_y*cos_r                     cos_y*cos_r                      sin_r       ][0]
            # [az (U)] = [sin_y*cos_p*sin_r+cos_y*sin_p    -cos_y*cos_p*sin_r+sin_y*sin_p   cos_p*cos_r ][g]
            # [ax (E)] = [-gsin_p*cos_r]
            # [ay (N)] = [gsin_r      ]
            # [az (U)] = [gcos_p*cos_r]
            roll = np.arctan2(ay, np.sqrt(ax**2 + az**2)) # or np.arcsin(ay)
            pitch = np.arctan2(-ax, az)
            yaw = 0.0
        elif nav=="NED": # ZYX (yaw - pitch - roll)
            # pitch limited between +- 90 degrees as Gimbal Lock problem (Singularity)
            # [ax (N)] = [cos_y*cos_p                     sin_y*cos_p                     -sin_p     ][0]
            # [ay (E)] = [cos_y*sin_p*sin_r-sin_y*cos_r   sin_y*sin_p*sin_r+cos_y*cos_r   cos_p*sin_r][0]
            # [az (D)] = [cos_y*sin_p*cos_r+sin_y*sin_r   sin_y*sin_p*cos_r-cos_y*sin_r   cos_p*cos_r][g]
            # [ax (N)] = [-gsin_p      ]
            # [ay (E)] = [ gcos_p*sin_r]
            # [az (D)] = [ gcos_p*cos_r]
            roll = np.arctan2(ay, az)
            pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2)) # or np.arcsin(-ax) 
            yaw = 0.0
        else:
            raise ValueError("Navigation frame should be either ENU or NED")
    return roll, pitch, yaw

def acc2quat(ax, ay, az, nav="ENU"):
    """
    Convert acceleration vector with gravity to Quaternion in ENU or NED frame

    :param float ax: x axis accelerometer
    :param float ay: y axis accelerometer
    :param float az: z axis accelerometer
    :param str nav: navigation frame
    :returns: 
        - w (float) - Quaternion magnitude
        - x (float) - Quaternion X axis
        - y (float) - Quaternion Y axis
        - z (float) - Quaternion Z axis
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
    Convert acceleration vector with gravity and magnetometer value to Euler angle in ENU or NED frame included tilt compensation[1]_

    :param float ax: x axis accelerometer
    :param float ay: y axis accelerometer
    :param float az: z axis accelerometer
    :param float mx: x axis gyroscope
    :param float my: y axis gyroscope
    :param float mz: z axis gyroscope
    :param str nav: navigation frame
    :returns: 
        - roll (float) - x-axis Euler angle in radians
        - pitch (float) - y-axis Euler angle in radians
        - yaw (float) - z-axis Euler angle in radians

    .. Reference
    .. [1] Page.163, Fundamentals of Inertial Navigation, Satellite-based Positioning and their Intergration, Springer, 2013 
    """
    roll, pitch, yaw = acc2eul(ax, ay, az, nav) # accelerometer provide roll and pitch angle
    mag = np.array([[mx],[my],[mz]])
    if np.linalg.norm(mag) > 0:
        mag_norm = np.linalg.norm(mag)
        mx = mx/mag_norm
        my = my/mag_norm
        mz = mz/mag_norm
        mag = np.array([[mx],[my],[mz]])
        if nav == "ENU":
            # mx, my, mz in body frame
            # bx, by, bz in navigation frame
            # bx (E) = mx*(-sin_y*sin_p*sin_r+cos_y*cos_p) + my*(-sin_y*cos_r) + mz*(sin_y*cos_p*sin_r+cos_y*sin_p)
            # by (N) = mx*(cos_y*sin_p*sin_r+sin_y*cos_p)  + my*(cos_y*cos_r)  + mz*(-cos_y*cos_p*sin_r+sin_y*sin_p)
            # bz (U) = mx*(-sin_p*cos_r)                   + my*(sin_r)        + mz*(cos_p*cos_r)

            # magnetometer reading in East axis will become 0, when body frame overlap with navigation frame
            # 0 = mx*(-sin_y*sin_p*sin_r+cos_y*cos_p) + my*(-sin_y*cos_r) + mz*(sin_y*cos_p*sin_r+cos_y*sin_p)
            yaw = np.arctan2(mx*np.cos(pitch) + mz*np.sin(pitch), mx*np.sin(pitch)*np.sin(roll) + my*np.cos(roll) - mz*np.cos(pitch)*np.sin(roll))
        elif nav == "NED":
            # mx, my, mz in body frame
            # bx, by, bz in navigation frame
            # bx (N) = mx*(cos_y*cos_p) + my*(cos_y*sin_p*sin_r-sin_y*cos_r)  + mz*(cos_y*sin_p*cos_r+sin_y*sin_r)
            # by (E) = mx*(sin_y*cos_p) + my*(sin_y*sin_p*sin_r+cos_y*cos_r)  + mz*(sin_y*sin_p*cos_r-cos_y*sin_r)
            # bz (D) = mx*(-sin_p)      + my*(cos_p*sin_r)                    + mz*(cos_p*cos_r)

            # magnetometer reading in East axis will become 0, when body frame overlap with navigation frame
            # 0 = mx*(sin_y*cos_p) + my*(sin_y*sin_p*sin_r+cos_y*cos_r)  + mz*(sin_y*sin_p*cos_r-cos_y*sin_r)
            yaw = -np.arctan2(my*np.cos(roll) - mz*np.sin(roll), mx*np.cos(pitch) + my*np.sin(pitch)*np.sin(roll) + mz*np.sin(pitch)*np.cos(roll))
        else:
            raise ValueError("Navigation frame should be either ENU or NED")
    return roll, pitch, yaw

def accmag2quat(ax, ay, az, mx, my, mz, nav="ENU"):
    """
    Convert acceleration vector with gravity and magnetometer value to Quaternion in ENU or NED frame

    :param float ax: x axis accelerometer
    :param float ay: y axis accelerometer
    :param float az: z axis accelerometer
    :param float mx: x axis magnetometer
    :param float my: y axis magnetometer
    :param float mz: z axis magnetometer
    :param str nav: navigation frame
    :returns: 
        - w (float) - Quaternion magnitude
        - x (float) - Quaternion X axis
        - y (float) - Quaternion Y axis
        - z (float) - Quaternion Z axis
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
    Convert data from ENU frame to NED frame

    :param float E: East axis value
    :param float N: North axis value
    :param float U: Upward value
    :returns: 
        - N (float) - North axis value
        - E (float) - East axis value
        - D (float) - Downward value
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
    Convert data from NED frame to ENU frame

    :param float N: North axis value
    :param float E: East axis value
    :param float D: Downward value
    :returns: 
        - E (float) - East axis value
        - N (float) - North axis value
        - U (float) - Upward value
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

def skew_symmetric(x, y, z):
    """
    Create skew symmetric matrix by vector

    :param float x: 1st element of vector
    :param float y: 2nd element of vector
    :param float z: 3rd element of vector
    :returns: 
        - matrix (ndarray) - skew-symmetric matrix
    """
    matrix = np.array([[0, -z, y],
                       [z, 0, -x],
                       [-y, x, 0.0]])
    return matrix