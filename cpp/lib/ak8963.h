#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <yaml-cpp/yaml.h>
#include "utils.h"
#include "transformation.h"

using namespace std;
using namespace Eigen;
using namespace YAML;

#define WIA 0x00
#define INFO 0x01
#define ST1 0x02
#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08
#define ST2 0x09
#define CNTL 0x0A
#define ASAX 0x10
#define ASAY 0x11
#define ASAZ 0x12

struct mag_calib{
    Vector3d scale;
    Vector3d bias;
    VectorXd misalignment;
    float strength;
};

class AK8963
{
    /**
     * @brief AK8963 I2C driver for acquire magnetometer data

     * @param i2c: AK8963 I2C connection port
     * @param nav_frame: navigation frame
     * @param hz: IMU frequency
     * @param calibration: calibrate magnetometer
     */

    private:
        int i2c;
        string nav_frame;
        int axis;
        float hz, dt;
        bool calibration;
        Node config;
        float mag_fs, mag_strength;
        float adjustment_x, adjustment_y, adjustment_z;
        Vector3d mag_bias, mag_scale;
        VectorXd mag_misalignment = VectorXd(6);
        float mx_bias, my_bias, mz_bias;
        float mx_scale = 1, my_scale = 1, mz_scale = 1;
        float mxy_mis = 0, mxz_mis = 0, myx_mis = 0, myz_mis = 0, mzx_mis = 0, mzy_mis = 0;

    public:
        AK8963(){}; // Default constructor
        AK8963(int i2c, string nav_frame="NED", float hz=100, bool calibration=false) { // Constructor with parameters
            this->i2c = i2c;
            this->nav_frame = nav_frame;
            this->hz = hz;
            this->dt = 1/hz;
            this->calibration = calibration;
            if (this->calibration == false)
            {
                this->config = LoadFile("../../cfg/config.yaml");
                this->mx_bias = config[nav_frame]["mx_bias"].as<float>();
                this->my_bias = config[nav_frame]["my_bias"].as<float>();
                this->mz_bias = config[nav_frame]["mz_bias"].as<float>();
                this->mag_bias << mx_bias, my_bias, mz_bias;
                
                this->mx_scale = config[nav_frame]["mx_scale"].as<float>();
                this->my_scale = config[nav_frame]["my_scale"].as<float>();
                this->mz_scale = config[nav_frame]["mz_scale"].as<float>();
                this->mag_scale << mx_scale, my_scale, mz_scale;
            }

            // Check parameter
            if ((this->nav_frame != "ENU") and (this->nav_frame != "NED"))
            {
                throw invalid_argument("Navigation frame should be either ENU or NED");
            }
        }

        void who_am_i();
        void config_AK8963(int mag_parameter);
        void get_status();
        void get_adjust_mag();
        mag_calib mag_calibration(float s);
        Vector3d get_mag();
        void set_mode(string mode, int bit);
        int read_raw_data(int high_register, int low_register);
        int read_8bit_register(int single_register);
};