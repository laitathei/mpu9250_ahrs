#pragma once
#include <iostream>
#include <cmath>
#include <thread>
#include <memory>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "mpu6500.h"
#include "ak8963.h"
#include "ahrs.h"
#include "transformation.h"

using namespace std;
using namespace Eigen;

class MPU9250
{
    /**
     * @brief MPU9250 I2C driver for accessing MPU6500 and AK8963

     * @param nav_frame: navigation frame
     * @param axis: axis data
     * @param hz: IMU frequency
     * @param calibration: calibrate gyroscope and accelerometer
     */
     
    private:
        int axis;
        int mpu6500_i2c, ak8963_i2c;
        int mpu6500_address = 0x68;
        int ak8963_address = 0x0c;
        int queue_size = 20;
        int window_size = 5;
        bool calibration;
        Vector3d euler, accel, gyro, mag;
        Vector4d quaternion;

    public:
        thread imu_th;
        unique_ptr<ahrs> ahrs_ptr;
        string nav_frame;
        float hz, dt;
        float temp, roll, pitch, yaw;
        float w, x, y, z;
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        string body_frame;
        string rotation_seq;
        MPU6500 mpu6500;
        AK8963 ak8963;
        MPU9250(){}; // Default constructor
        MPU9250(string nav_frame, int axis, float hz, bool calibration) { // Constructor with parameters
            this->nav_frame = nav_frame;
            this->axis = axis;
            this->hz = hz;
            this->dt = 1/hz;
            this->calibration = calibration;

            this->mpu6500_i2c = wiringPiI2CSetup(mpu6500_address);
            this->ak8963_i2c = wiringPiI2CSetup(ak8963_address);

            cout << "Default MPU6500 address is 0x68" << endl;
            cout << "Default AK8963 address is 0x0c" << endl;

            if ((this->axis != 6) and (this->axis != 9)){
                throw invalid_argument("Axis must be 6 or 9");
            }
            if (this->nav_frame == "NED"){
                this->body_frame = "FRD";
                this->rotation_seq = "zyx";
            }
            else if (this->nav_frame == "ENU"){
                this->body_frame = "RFU";
                this->rotation_seq = "zxy";
            }
            else{
                throw invalid_argument("Navigation frame should be either ENU or NED");
            }

            // Config MPU6500
            MPU6500 mpu6500_config(this->mpu6500_i2c, this->nav_frame, this->hz, this->calibration);
            mpu6500_config.control_accel_gyro(true, true, true, true, true, true);
            mpu6500_config.who_am_i();
            mpu6500_config.config_MPU6500(0, 0);
            this->mpu6500 = mpu6500_config;

            // Config AK8963
            AK8963 ak8963_config(this->ak8963_i2c, this->nav_frame, this->hz, this->calibration);
            ak8963_config.who_am_i();
            ak8963_config.config_AK8963(16);
            this->ak8963 = ak8963_config;
        }

        Vector3d get_accel();
        Vector3d get_gyro();
        Vector3d get_mag();
        float get_temp();
        Vector3d get_euler();
        Vector4d get_quaternion();
        Vector3d get_ahrs_euler();
        Vector4d get_ahrs_quaternion();
        void initialization();
        void start_thread(unique_ptr<ahrs> ahrs_ptr=nullptr);
        void run();
};