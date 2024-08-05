#include <cmath>
#include "madgwick.h"

void Madgwick::init_quat(float w, float x, float y, float z)
{
    this->est_quat << w, x, y, z;
}

Vector4d Madgwick::run(Vector3d acc, Vector3d gyr, Vector3d mag, float hz)
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

Vector4d Madgwick::gyro_acc_fusion()
{
    Vector4d origin_q, quat_diff, quat_change;
    Vector3d f;
    MatrixXd J(3, 4);
    float qw, qx, qy, qz;
    float ax, ay, az;
    float gx, gy, gz;
    Vector3d acc = this->acc;
    Vector3d gyr = this->gyr;
    gx = gyr[0], gy = gyr[1], gz = gyr[2];
    float a_norm = acc.norm();
    origin_q << 0, gx, gy, gz; // the current reading of gyroscope in quaternion form
    quat_diff = quat_multi(this->est_quat, origin_q);
    quat_change = 0.5 * quat_diff; // compute the rate of change of quaternion
    if (a_norm > 0)
    {
        Vector3d a = acc/a_norm; // normalize acceleration vector
        this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
        qw = this->est_quat[0], qx = this->est_quat[1], qy = this->est_quat[2], qz = this->est_quat[3];
        ax = a[0], ay = a[1], az = a[2];
        f << 2.0*(qx*qz - qw*qy) - ax, // calculate objective function
             2.0*(qw*qx + qy*qz) - ay,
             2.0*(0.5-pow(qx,2)-pow(qy,2)) - az;
        J << -2.0*qy, 2.0*qz, -2.0*qw, 2.0*qx, // calculate Jacobian form
              2.0*qx,  2.0*qw, 2.0*qz,  2.0*qy,
              0.0,    -4.0*qx, -4.0*qy, 0.0;
        Vector4d gradient = J.transpose() * f;
        float gradient_norm = gradient.norm();
        gradient = gradient/gradient_norm; // normalize gradient
        quat_change = quat_change - this->gain * gradient; // compute new quaternion after gradient descent
    }
    this->est_quat = this->est_quat + quat_change * this->imu_dt; // update the quaternion
    this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
    return this->est_quat;
}

Vector4d Madgwick::gyro_acc_mag_fusion()
{
    Vector3d b;
    Vector4d origin_q, quat_diff, quat_change, h;
    VectorXd m(3);
    float qw, qx, qy, qz;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float bx, by, bz;
    Vector3d acc = this->acc;
    Vector3d gyr = this->gyr;
    Vector3d mag = this->mag;
    gx = gyr[0], gy = gyr[1], gz = gyr[2];
    float a_norm = acc.norm();
    float m_norm = mag.norm();
    origin_q << 0, gx, gy, gz; // the current reading of gyroscope in quaternion form
    quat_diff = quat_multi(this->est_quat, origin_q);
    quat_change = 0.5 * quat_diff; // compute the rate of change of quaternion
    if ((a_norm > 0) and (m_norm > 0))
    {
        Vector3d a = acc/a_norm; // normalize acceleration vector
        m = mag/m_norm; // normalize magnetometer vector
        ax = a[0], ay = a[1], az = a[2];
        mx = m[0], my = m[1], mz = m[2];
        m.conservativeResize(4);
        m << 0, mx, my, mz;
        h << quat_multi(this->est_quat, quat_multi(m, quat_conjugate(this->est_quat)));
        this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
        qw = this->est_quat[0], qx = this->est_quat[1], qy = this->est_quat[2], qz = this->est_quat[3];
        if (this->nav_frame == "ENU"){
            // In ENU frame, Y axis in body frame align to north, X axis will 0
            b << 0, sqrt(pow(h[1],2)+pow(h[2],2)), h[3];
            bx = b[0], by = b[1], bz = b[2];
        }
        else if (this->nav_frame == "NED"){
            // In NED frame, X axis in body frame align to north, Y axis will 0
            b << sqrt(pow(h[1],2)+pow(h[2],2)), 0, h[3];
            bx = b[0], by = b[1], bz = b[2];
        }
        VectorXd f(6); // calculate objective function
        f << 2.0*(qx*qz - qw*qy) - ax,
            2.0*(qw*qx + qy*qz) - ay,
            2.0*(0.5-pow(qx,2)-pow(qy,2)) - az,
            2.0*bx*(0.5 - pow(qy,2) - pow(qz,2)) + 2.0*by*(qw*qz + qx*qy) + 2.0*bz*(qx*qz - qw*qy) - mx,
            2.0*bx*(qx*qy - qw*qz) + 2.0*by*(0.5 - pow(qx,2) - pow(qz,2)) + 2.0*bz*(qw*qx + qy*qz) - my,
            2.0*bx*(qw*qy + qx*qz) + 2.0*by*(qy*qz - qw*qx) + 2.0*bz*(0.5 - pow(qx,2) - pow(qy,2)) - mz;
        MatrixXd J(6,4); // calculate Jacobian form
        J << -2.0*qy, 2.0*qz, -2.0*qw, 2.0*qx, 
            2.0*qx, 2.0*qw, 2.0*qz, 2.0*qy,
            0.0, -4.0*qx, -4.0*qy, 0.0,
            2.0*by*qz-2.0*bz*qy, 2.0*by*qy+2.0*bz*qz, -4.0*bx*qy+2.0*by*qx-2.0*bz*qw, -4.0*bx*qz+2.0*by*qw+2.0*bz*qx,
            -2.0*bx*qz+2.0*bz*qx, 2.0*bx*qy-4.0*by*qx+2.0*bz*qw, 2.0*bx*qx+2.0*bz*qz, -2.0*bx*qw-4.0*by*qz+2.0*bz*qy,
            2.0*bx*qy-2.0*by*qx, 2.0*bx*qz-2.0*by*qw-4.0*bz*qx, 2.0*bx*qw+2.0*by*qz-4.0*bz*qy, 2.0*bx*qx+2.0*by*qy;
        Vector4d gradient = J.transpose() * f;
        float gradient_norm = gradient.norm();
        gradient = gradient/gradient_norm; // normalize gradient
        quat_change = quat_change - this->gain * gradient; // compute new quaternion after gradient descent
    }
    this->est_quat = this->est_quat + quat_change * this->imu_dt; // update the quaternion
    this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
    return this->est_quat;
}