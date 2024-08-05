#include <cmath>
#include "mahony.h"

void Mahony::init_quat(float w, float x, float y, float z)
{
    this->est_quat << w, x, y, z;
}

Vector4d Mahony::run(Vector3d acc, Vector3d gyr, Vector3d mag, float hz)
{
    if (acc.rows() != 3 || acc.cols() != 1) {
        throw invalid_argument("acc shape must be (3,1)");
    } else if (gyr.rows() != 3 || gyr.cols() != 1) {
        throw invalid_argument("gyr shape must be (3,1)");
    } else if (mag.rows() != 3 || mag.cols() != 1) {
        throw invalid_argument("mag shape must be (3,1)");
    }
    this->imu_hz = hz;
    this->imu_dt = 1/this->imu_hz;
    this->acc = acc;
    this->gyr = gyr;
    this->mag = mag;
    if (this->axis == 6){ // 6 axis fusion (gyroscope and accelerometer)
        this->est_quat = this->gyro_acc_fusion();
    }
    else if (this->axis == 9){ // 9 axis fusion (gyroscope, accelerometer and magnetometer)
        this->est_quat = this->gyro_acc_mag_fusion();
    }
    return this->est_quat;
}

Vector4d Mahony::gyro_acc_fusion()
{
    Vector3d v_g;
    Vector4d origin_q, quat_diff, quat_change, h;
    float qw, qx, qy, qz;
    float gx, gy, gz;
    Vector3d acc = this->acc;
    Vector3d gyr = this->gyr;
    gx = gyr[0], gy = gyr[1], gz = gyr[2];
    qw = this->est_quat[0], qx = this->est_quat[1], qy = this->est_quat[2], qz = this->est_quat[3];
    float a_norm = acc.norm();
    Matrix3d DCM = quat2dcm(qw, qx, qy, qz); // DCM from body frame to navigation frame
    if (a_norm > 0)
    {
        // accelerometer part
        Vector3d a = acc/a_norm; // normalize acceleration vector 
        if (this->nav_frame == "ENU"){
            Vector3d g(0,0,1);
            v_g = DCM.transpose() * g; // convert expected gravity vector from navigation frame to body frame
        }
        else if (this->nav_frame == "NED"){
            Vector3d g(0,0,-1);
            v_g = DCM.transpose() * g; // convert expected gravity vector from navigation frame to body frame
        }
        Vector3d acc_error = a.cross(v_g); // error of accelerometer (gravity vector in body frame cross prodcut with accelerometer vector)
        
        // Mahony algorithm
        Vector3d total_error = acc_error;
        this->gyro_bias = this->gyro_bias + this->ki * total_error * this->imu_dt; // estimate current gyroscope bias by adding past changes
        gyr = gyr + this->gyro_bias + this->kp * total_error; // gyroscope correction
        gx = gyr[0], gy = gyr[1], gz = gyr[2];
    }
    origin_q << 0, gx, gy, gz; // the current reading of gyroscope in quaternion form
    quat_diff = quat_multi(this->est_quat, origin_q);
    quat_change = 0.5 * quat_diff; // compute the rate of change of quaternion
    this->est_quat = this->est_quat + quat_change * this->imu_dt; // update the quaternion
    this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
    return this->est_quat;
}

Vector4d Mahony::gyro_acc_mag_fusion()
{
    Vector3d v_g, v_m, b, h;
    Vector4d origin_q, quat_diff, quat_change;
    float qw, qx, qy, qz;
    float gx, gy, gz;
    float mx, my, mz;
    Vector3d acc = this->acc;
    Vector3d gyr = this->gyr;
    Vector3d mag = this->mag;
    gx = gyr[0], gy = gyr[1], gz = gyr[2];
    qw = this->est_quat[0], qx = this->est_quat[1], qy = this->est_quat[2], qz = this->est_quat[3];
    float a_norm = acc.norm();
    float m_norm = mag.norm();
    Matrix3d DCM = quat2dcm(qw, qx, qy, qz); // DCM from body frame to navigation frame
    if ((a_norm > 0) and (m_norm > 0))
    {
        // accelerometer part
        Vector3d a = acc/a_norm; // normalize acceleration vector 
        if (this->nav_frame == "ENU"){
            Vector3d g(0,0,1);
            v_g = DCM.transpose() * g; // convert expected gravity vector from navigation frame to body frame
        }
        else if (this->nav_frame == "NED"){
            Vector3d g(0,0,-1);
            v_g = DCM.transpose() * g; // convert expected gravity vector from navigation frame to body frame
        }
        Vector3d acc_error = a.cross(v_g); // error of accelerometer (gravity vector in body frame cross prodcut with accelerometer vector)
        
        // magnetometer part
        Vector3d m = mag/m_norm; // normalize magnetometer vector
        h = DCM.transpose() * m; // convert magnetic field from body frame to navigation frame
        if (this->nav_frame == "ENU"){
            // In ENU frame, Y axis in body frame align to north, X axis will 0
            b << 0, sqrt(pow(h[0],2)+pow(h[1],2)), h[2];
        }
        else if (this->nav_frame == "NED"){
            // In NED frame, X axis in body frame align to north, Y axis will 0
            b << sqrt(pow(h[0],2)+pow(h[1],2)), 0, h[2];
        }
        v_m = DCM.transpose() * b; // convert the aligned magnetometer vector from navigation frame to body frame
        Vector3d mag_error = m.cross(v_m); // error of magnetometer (aligned magnetometer vector in body frame cross prodcut with origin magnetometer vector)

        // Mahony algorithm
        Vector3d total_error = acc_error + mag_error;
        this->gyro_bias = this->gyro_bias + this->ki * total_error * this->imu_dt; // estimate current gyroscope bias by adding past changes
        gyr = gyr + this->gyro_bias + this->kp * total_error; // gyroscope correction
        gx = gyr[0], gy = gyr[1], gz = gyr[2];
    }
    origin_q << 0, gx, gy, gz; // the current reading of gyroscope in quaternion form
    quat_diff = quat_multi(this->est_quat, origin_q);
    quat_change = 0.5 * quat_diff; // compute the rate of change of quaternion
    this->est_quat = this->est_quat + quat_change * this->imu_dt; // update the quaternion
    this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
    return this->est_quat;
}