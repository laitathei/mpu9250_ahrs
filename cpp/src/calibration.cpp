#include <iostream>
#include <fstream>
#include <map>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "../lib/utils.h"
#include "../lib/orientation.h"
#include "../lib/transformation.h"
#include "../lib/mpu9250.h"

using namespace std;
using namespace Eigen;
using namespace YAML;

int main(int argc, char **argv)
{
    string nav_frame = "NED"; // ENU/NED
    int axis = 9;
    float hz = 100;
    float acc_time = 1;
    float gyro_time = 1;
    float mag_time = 10;
    bool calibration = true;
    accel_gyro_calib accel_result, gyro_result;
    mag_calib mag_result;
    float mag_strength;
    Vector3d accel_scale, accel_bias, gyro_scale, gyro_bias, mag_scale, mag_bias;
    VectorXd bias = VectorXd(9);
    VectorXd scale = VectorXd(9);
    VectorXd accel_misalignment = VectorXd(6);
    VectorXd gyro_misalignment = VectorXd(6);
    VectorXd mag_misalignment = VectorXd(6);

    MPU9250 mpu9250(nav_frame, axis, hz, calibration);
    mpu9250.initialization();
    gyro_result = mpu9250.mpu6500.gyro_calibration(gyro_time);
    gyro_scale = gyro_result.scale;
    gyro_bias = gyro_result.bias;
    gyro_misalignment = gyro_result.misalignment;
    cout << "gyro_scale:" << endl;
    cout << gyro_scale << endl;
    cout << "gyro_bias:" << endl;
    cout << gyro_bias << endl;
    cout << "gyro_misalignment:" << endl;
    cout << gyro_misalignment << endl;

    accel_result = mpu9250.mpu6500.accel_calibration(acc_time);
    accel_scale = accel_result.scale;
    accel_bias = accel_result.bias;
    accel_misalignment = accel_result.misalignment;
    cout << "accel_scale:" << endl;
    cout << accel_scale << endl;
    cout << "accel_bias:" << endl;
    cout << accel_bias << endl;
    cout << "accel_misalignment:" << endl;
    cout << accel_misalignment << endl;
    
    mag_result = mpu9250.ak8963.mag_calibration(mag_time);
    mag_scale = mag_result.scale;
    mag_bias = mag_result.bias;
    mag_misalignment = mag_result.misalignment;
    mag_strength = mag_result.strength;
    cout << "mag_scale:" << endl;
    cout << mag_scale << endl;
    cout << "mag_bias:" << endl;
    cout << mag_bias << endl;
    cout << "mag_misalignment:" << endl;
    cout << mag_misalignment << endl;
    cout << "mag_strength:" << endl;
    cout << mag_strength << endl;

    bias << gyro_bias, accel_bias, mag_bias;
    scale << gyro_scale, accel_scale, mag_scale;

    // Refresh new config to yaml file
    Node config = LoadFile("../../cfg/config.yaml");
    vector<string> bias_parameter = {"gx_bias", "gy_bias", "gz_bias", "ax_bias", "ay_bias", "az_bias", "mx_bias", "my_bias", "mz_bias"};
    vector<string> scale_parameter = {"gx_scale", "gy_scale", "gz_scale", "ax_scale", "ay_scale", "az_scale", "mx_scale", "my_scale", "mz_scale"};
    map<string, float> bias_scale;

    for (int i=0; i < bias_parameter.size(); i++) {
        bias_scale[bias_parameter[i]] = bias[i];
    }
    for (int i=0; i < scale_parameter.size(); i++) {
        bias_scale[scale_parameter[i]] = scale[i];
    }

    config[nav_frame] = bias_scale;
    ofstream fout("../../cfg/config.yaml");
    fout << config;

    return 0;
}