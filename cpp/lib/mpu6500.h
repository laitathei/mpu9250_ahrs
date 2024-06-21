#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;
using namespace YAML;

#define SMPLRT_DIV 0x19
#define BYPASS_ENABLE 0x37
#define GYRO_CONFIG_2 0x1a
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1c
#define ACCEL_CONFIG_2 0x1d
#define ACCEL_XOUT_H 0x3b
#define ACCEL_XOUT_L 0x3c
#define ACCEL_YOUT_H 0x3d
#define ACCEL_YOUT_L 0x3e
#define ACCEL_ZOUT_H 0x3f
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6b
#define PWR_MGMT_2 0x6c
#define WHO_AM_I 0x75 // default return 0x71

#define ACCEL_FS_SEL_2G 0 // 0b00000000
#define ACCEL_FS_SEL_4G 8 // 0b00001000
#define ACCEL_FS_SEL_8G 16 // 0b00010000
#define ACCEL_FS_SEL_16G 24 // 0b00011000

#define GYRO_FS_SEL_250DPS 0 // 0b00000000
#define GYRO_FS_SEL_500DPS 8 // 0b00001000
#define GYRO_FS_SEL_1000DPS 16 // 0b00010000
#define GYRO_FS_SEL_2000DPS 24 // 0b00011000

// From MPU9250 datasheet 3.4.2
#define TEMP_SENSITIVITY 333.87
#define ROOM_TEMP_OFFSET 0

// standard acceleration of gravity
#define g 9.80665

struct accel_gyro_calib{
    Vector3d scale;
    Vector3d bias;
    VectorXd misalignment;
};

class MPU6500
{
    /**
     * @brief MPU6500 I2C driver for acquire accelerometer and gyroscope data

     * @param i2c: MPU6500 I2C connection port
     * @param nav_frame: navigation frame
     * @param hz: IMU frequency
     * @param calibration: calibrate gyroscope and accelerometer
     */

    private:
        int i2c;
        string nav_frame;
        int axis;
        float hz, dt;
        bool calibration;
        Node config;
        float accel_fs, gyro_fs;
        Vector3d accel_bias, accel_scale;
        Vector3d gyro_bias, gyro_scale;
        VectorXd accel_misalignment = VectorXd(6);
        VectorXd gyro_misalignment = VectorXd(6);
        float ax, ay, az, gx, gy, gz, temp;
        float ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias;
        float ax_scale = 1, ay_scale = 1, az_scale = 1, gx_scale = 1, gy_scale = 1, gz_scale = 1;
        float axy_mis = 0, axz_mis = 0, ayx_mis = 0, ayz_mis = 0, azx_mis = 0, azy_mis = 0;
        float gxy_mis = 0, gxz_mis = 0, gyx_mis = 0, gyz_mis = 0, gzx_mis = 0, gzy_mis = 0;

    public:
        MPU6500(){}; // Default constructor
        MPU6500(int i2c, string nav_frame="ENU", float hz=100, bool calibration=false) { // Constructor with parameters
            this->i2c = i2c;
            this->nav_frame = nav_frame;
            this->hz = hz;
            this->dt = 1/hz;
            this->calibration = calibration;

            if (this->calibration == false)
            {
                this->config = LoadFile("../../cfg/config.yaml");
                this->ax_bias = config[nav_frame]["ax_bias"].as<float>();
                this->ay_bias = config[nav_frame]["ay_bias"].as<float>();
                this->az_bias = config[nav_frame]["az_bias"].as<float>();
                this->accel_bias << ax_bias, ay_bias, az_bias;

                this->gx_bias = config[nav_frame]["gx_bias"].as<float>();
                this->gy_bias = config[nav_frame]["gy_bias"].as<float>();
                this->gz_bias = config[nav_frame]["gz_bias"].as<float>();
                this->gyro_bias << gx_bias, gy_bias, gz_bias;

                this->ax_scale = config[nav_frame]["ax_scale"].as<float>();
                this->ay_scale = config[nav_frame]["ay_scale"].as<float>();
                this->az_scale = config[nav_frame]["az_scale"].as<float>();
                this->accel_scale << ax_scale, ay_scale, az_scale;

                this->gx_scale = config[nav_frame]["gx_scale"].as<float>();
                this->gy_scale = config[nav_frame]["gy_scale"].as<float>();
                this->gz_scale = config[nav_frame]["gz_scale"].as<float>();
                this->gyro_scale << gx_scale, gy_scale, gz_scale;
            }

            // Check parameter
            if ((this->nav_frame != "ENU") and (this->nav_frame != "NED"))
            {
                throw invalid_argument("Navigation frame should be either ENU or NED");
            }
        }

        void control_accel_gyro(bool ax, bool ay, bool az, bool gx, bool gy, bool gz);
        void who_am_i();
        void config_MPU6500(int accel_parameter, int gyro_parameter);
        int read_raw_data(int high_register, int low_register);
        int read_8bit_register(int single_register);
        accel_gyro_calib gyro_calibration(float s);
        accel_gyro_calib accel_calibration(float s);
        Vector3d get_accel();
        Vector3d get_gyro();
        float get_temp();
};