#!/usr/bin/python3
import numpy as np
from utils.orientation import quat_multi, quat_conjugate

class Madgwick():
    """
    Madgwick is one of the AHRS filter applied with gradient descent technique [1]_

    :param int axis: axis data for fusion
    :param float gain: 6/9 axis fusion gain
    :param str nav_frame: navigation frame

    .. Reference
    .. [1] 'Madgwick <https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#orientation-from-angular-rate>'
    """
    def __init__(self, axis, gain, nav_frame="NED"):
        # Body frame is front(X)-right(Y)-down(Z), Navigation frame is NED
        # Body frame is front(Y)-right(X)-down(Z), Navigation frame is ENU

        # algorithm parameter
        self.axis = axis # 6 or 9
        # Increasing the gain makes the filter respond quickly to changes, but it also causes the filter to become sensitive to noise.
        # Reducing the gain makes the filter take more time to converge, which means a delay in calculating the attitude
        # increase gain means trust more accelerometers and magnetometers, decreasing means trust more gyroscopes
        self.gain = gain # 6 axis default gain 0.033, 9 axis default gain 0.041

        self.nav_frame = nav_frame # ENU or NED
        if self.nav_frame != "ENU" and self.nav_frame != "NED":
            raise ValueError("Navigation frame must be ENU or NED")

        if self.axis != 6 and self.axis != 9:
            raise ValueError("Axis must be 6 or 9")

        print("Madgwick filter in use")
        
    def init_quat(self, w, x, y, z):
        """
        Madgwick filter initial attitude

        :param float w: Quaternion magnitude
        :param float x: Quaternion X axis
        :param float y: Quaternion Y axis
        :param float z: Quaternion Z axis
        """
        self.est_quat = np.array([[w],[x],[y],[z]])

    def run(self, acc, gyr, mag, hz):
        """
        Iteration of Madgwick filter

        :param ndarray acc: accelerometer data
        :param ndarray gyr: gyroscope data
        :param ndarray mag: magnetometer data
        :param int hz: IMU frequency
        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        if acc.shape != (3,1):
            raise ValueError("acc shape must be (3,1)")
        elif gyr.shape != (3,1):
            raise ValueError("gyr shape must be (3,1)")
        elif mag.shape != (3,1):
            raise ValueError("mag shape must be (3,1)")

        self.imu_hz = hz
        self.imu_dt = 1/self.imu_hz
        self.acc = acc
        self.gyr = gyr
        self.mag = mag
        if self.axis == 6: # 6 axis fusion (gyroscope and accelerometer)
            w, x, y, z = self.gyro_acc_fusion()
        elif self.axis == 9: # 9 axis fusion (gyroscope, accelerometer and magnetometer)
            w, x, y, z = self.gyro_acc_mag_fusion()
        return w, x, y, z

    def gyro_acc_fusion(self):
        """
        Madgwick filter 6 axis data fusion

        ENU: \n
        Gravity is defined as negative when pointing upwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n

        NED: \n
        Gravity is defined as negative when pointing downwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n

        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        acc = np.copy(self.acc)
        gyr = np.copy(self.gyr)
        gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        a_norm = np.linalg.norm(acc)
        origin_q = np.array([[0],[gx],[gy],[gz]]) # the current reading of gyroscope in quaternion form
        quat_diff = quat_multi(self.est_quat, origin_q)
        quat_change = 0.5 * quat_diff # compute the rate of change of quaternion
        if a_norm > 0:
            a = acc/a_norm # normalize acceleration vector
            self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
            qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
            ax, ay, az = a[0][0], a[1][0], a[2][0]
            f = np.array([[2.0*(qx*qz - qw*qy) - ax],     # calculate objective function
                        [2.0*(qw*qx + qy*qz) - ay],
                        [2.0*(0.5-qx**2-qy**2) - az]])
            J = np.array([[-2.0*qy, 2.0*qz, -2.0*qw, 2.0*qx],   # calculate Jacobian form
                          [2.0*qx,  2.0*qw, 2.0*qz,  2.0*qy],
                          [0.0,    -4.0*qx, -4.0*qy, 0.0]])
            gradient = J.T@f
            gradient_norm = np.linalg.norm(gradient)
            gradient = gradient/gradient_norm # normalize gradient
            quat_change = quat_change - self.gain * gradient # compute new quaternion after gradient descent
        self.est_quat = self.est_quat + quat_change * self.imu_dt # update the quaternion
        self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
        w, x, y, z = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return w, x, y, z

    def gyro_acc_mag_fusion(self):
        """
        Madgwick filter 9 axis data fusion

        ENU: \n
        Gravity is defined as negative when pointing upwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n
        Magnetometer data in Earth's reference (µT) \n

        NED: \n
        Gravity is defined as negative when pointing downwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n
        Magnetometer data in Earth's reference (µT) \n

        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        acc = np.copy(self.acc)
        gyr = np.copy(self.gyr)
        mag = np.copy(self.mag)
        gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        a_norm = np.linalg.norm(acc)
        m_norm = np.linalg.norm(mag)
        origin_q = np.array([[0],[gx],[gy],[gz]]) # the current reading of gyroscope in quaternion form
        quat_diff = quat_multi(self.est_quat, origin_q)
        quat_change = 0.5 * quat_diff # compute the rate of change of quaternion
        if a_norm > 0 and m_norm > 0:
            a = acc/a_norm # normalize acceleration vector
            m = mag/m_norm # normalize magnetometer vector
            ax, ay, az = a[0][0], a[1][0], a[2][0]
            mx, my, mz = m[0][0], m[1][0], m[2][0]
            m = np.array([[0],[mx],[my],[mz]])
            h = quat_multi(self.est_quat, quat_multi(m, quat_conjugate(self.est_quat)))               # (eq. 45)
            self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
            qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
            if self.nav_frame == "ENU":
                # In ENU frame, Y axis in body frame align to north, X axis will 0
                b = np.array([[0], [np.linalg.norm([h[1][0], h[2][0]])], [h[3][0]]])
                bx, by, bz = b[0][0], b[1][0], b[2][0]
            elif self.nav_frame == "NED":
                # In NED frame, X axis in body frame align to north, Y axis will 0
                b = np.array([[np.linalg.norm([h[1][0], h[2][0]])], [0], [h[3][0]]])
                bx, by, bz = b[0][0], b[1][0], b[2][0]
            f = np.array([[2.0*(qx*qz - qw*qy) - ax],     # calculate objective function
                        [2.0*(qw*qx + qy*qz) - ay],
                        [2.0*(0.5-qx**2-qy**2) - az],
                        [2.0*bx*(0.5 - qy**2 - qz**2) + 2.0*by*(qw*qz + qx*qy)         + 2.0*bz*(qx*qz - qw*qy)       - mx],
                        [2.0*bx*(qx*qy - qw*qz)       + 2.0*by*(0.5 - qx**2 - qz**2)   + 2.0*bz*(qw*qx + qy*qz)       - my],
                        [2.0*bx*(qw*qy + qx*qz)       + 2.0*by*(qy*qz - qw*qx)         + 2.0*bz*(0.5 - qx**2 - qy**2) - mz]])
            J = np.array([[-2.0*qy,                2.0*qz,                          -2.0*qw,                          2.0*qx],       # calculate Jacobian form
                          [ 2.0*qx,                2.0*qw,                          2.0*qz,                           2.0*qy],
                          [ 0.0,                   -4.0*qx,                         -4.0*qy,                          0.0],
                          [ 2.0*by*qz-2.0*bz*qy,   2.0*by*qy+2.0*bz*qz,             -4.0*bx*qy+2.0*by*qx-2.0*bz*qw,   -4.0*bx*qz+2.0*by*qw+2.0*bz*qx],
                          [-2.0*bx*qz+2.0*bz*qx,   2.0*bx*qy-4.0*by*qx+2.0*bz*qw,   2.0*bx*qx+2.0*bz*qz,              -2.0*bx*qw-4.0*by*qz+2.0*bz*qy],
                          [ 2.0*bx*qy-2.0*by*qx,   2.0*bx*qz-2.0*by*qw-4.0*bz*qx,   2.0*bx*qw+2.0*by*qz-4.0*bz*qy,    2.0*bx*qx+2.0*by*qy]])
            gradient = J.T@f
            gradient_norm = np.linalg.norm(gradient)
            gradient = gradient/gradient_norm # normalize gradient
            quat_change = quat_change - self.gain * gradient # compute new quaternion after gradient descent
        self.est_quat = self.est_quat + quat_change * self.imu_dt # update the quaternion
        self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
        w, x, y, z = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return w, x, y, z