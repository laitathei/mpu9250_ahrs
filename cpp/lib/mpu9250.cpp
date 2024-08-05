#include <cmath>
#include "mpu9250.h"
#include "orientation.h"

Vector3d MPU9250::get_accel()
{
    this->accel = this->mpu6500.get_accel();
    this->ax = accel[0];
    this->ay = accel[1];
    this->az = accel[2];
    return this->accel;
}

Vector3d MPU9250::get_gyro()
{
    this->gyro = this->mpu6500.get_gyro();
    this->gx = gyro[0];
    this->gy = gyro[1];
    this->gz = gyro[2];
    return this->gyro;
}

Vector3d MPU9250::get_mag()
{
    this->mag = this->ak8963.get_mag();
    this->mx = mag[0];
    this->my = mag[1];
    this->mz = mag[2];
    return this->mag;
}

float MPU9250::get_temp()
{
    this->temp = this->mpu6500.get_temp();
    return this->temp;
}

Vector3d MPU9250::get_euler()
{
    if (this->axis == 6){
        this->euler = acc2eul(this->ax, this->ay, this->az, this->nav_frame);
    }
    else if (this->axis == 9){
        this->euler = accmag2eul(this->ax, this->ay, this->az, this->mx, this->my, this->mz, this->nav_frame);
    }
    this->roll = this->euler[0] * 180.0 / M_PI;
    this->pitch = this->euler[1] * 180.0 / M_PI;
    this->yaw = this->euler[2] * 180.0 / M_PI;
    this->euler << this->roll, this->pitch, this->yaw;
    return this->euler;
}

Vector3d MPU9250::get_ahrs_euler()
{
    this->quaternion = this->ahrs_ptr->run(this->accel, this->gyro, this->mag, this->hz);
    this->w = this->quaternion[0], this->x = this->quaternion[1], this->y = this->quaternion[2], this->z = this->quaternion[3];
    this->euler = quat2eul(this->w, this->x, this->y, this->z, this->rotation_seq);
    this->roll = this->euler[0] * 180.0 / M_PI;
    this->pitch = this->euler[1] * 180.0 / M_PI;
    this->yaw = this->euler[2] * 180.0 / M_PI;
    this->euler << this->roll, this->pitch, this->yaw;
    return this->euler;
}

Vector4d MPU9250::get_quaternion()
{
    if (this->axis == 6){
        this->quaternion = acc2quat(this->ax, this->ay, this->az, this->nav_frame);
    }
    else if (this->axis == 9){
        this->quaternion = accmag2quat(this->ax, this->ay, this->az, this->mx, this->my, this->mz, this->nav_frame);
    }
    this->w = this->quaternion[0];
    this->x = this->quaternion[1];
    this->y = this->quaternion[2];
    this->z = this->quaternion[3];
    return this->quaternion;
}

Vector4d MPU9250::get_ahrs_quaternion()
{
    this->quaternion = this->ahrs_ptr->run(this->accel, this->gyro, this->mag, this->hz);
    this->w = this->quaternion[0];
    this->x = this->quaternion[1];
    this->y = this->quaternion[2];
    this->z = this->quaternion[3];
    return this->quaternion;
}

void MPU9250::initialization()
{
    this->temp = this->get_temp();
    this->accel = this->get_accel();
    this->gyro = this->get_gyro();
    this->mag = this->get_mag();
    this->euler = this->get_euler();
    this->quaternion = this->get_quaternion();
}

void MPU9250::start_thread(unique_ptr<ahrs> ahrs_ptr)
{
    if (ahrs_ptr != nullptr) {
        this->ahrs_ptr = move(ahrs_ptr);
        this->ahrs_ptr->init_quat(this->w, this->x, this->y, this->z);
    } else {
        this->ahrs_ptr = nullptr;
    }
    this->imu_th = thread(&MPU9250::run, this);
}

void MPU9250::run()
{
    while(1)
    {
        this->euler = this->get_accel();
        this->gyro = this->get_gyro();
        this->mag = this->get_mag();
        this->temp = this->get_temp();
        if (this->ahrs_ptr != nullptr){
            this->euler = this->get_ahrs_euler();
            this->quaternion = this->get_ahrs_quaternion();
            // cout << "trigger ahrs!" << endl;
        }
        else{
            this->euler = this->get_euler();
            this->quaternion = this->get_quaternion();
            // cout << "Not trigger ahrs!" << endl;
        }
    }
    this->imu_th.join();
}
