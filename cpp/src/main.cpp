#include <iostream>
#include <fstream>
#include <cmath>
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
    // float roll = 30* 3.14 /180;
    // float pitch = 60* 3.14 /180;
    // float yaw = 90* 3.14 /180;
    // Vector3d EulerAngle;
    // EulerAngle << roll, pitch, yaw;
    // EulerAngle = EulerAngle * 3.14 /180;
    // cout << EulerAngle << endl;

    // cout << EulerAngle.rows() << endl;
    // cout << EulerAngle.cols() << endl;

    // Matrix3d euler_x_rotation;
    // Matrix3d euler_y_rotation;
    // Matrix3d euler_z_rotation;
    // Matrix3d dcm;
    // euler_x_rotation << 1,0,0,
    //                     0,cos(roll),-sin(roll),
    //                     0,sin(roll),cos(roll);
    // euler_y_rotation << cos(pitch),0,sin(pitch),
    //                     0,1,0,
    //                     -sin(pitch),0,cos(pitch);
    // euler_z_rotation << cos(yaw),-sin(yaw),0,
    //                     sin(yaw),cos(yaw),0,
    //                     0,0,1;
    // dcm = euler_x_rotation * euler_y_rotation * euler_z_rotation;
    // cout << euler_x_rotation << endl;
    // cout << euler_y_rotation << endl;
    // cout << euler_z_rotation << endl;
    // cout << dcm << endl;
    // right_hand_rule right;
    // euler_x_rotation = right.euler_x_rotation(roll);
    // euler_y_rotation = right.euler_y_rotation(pitch);
    // euler_z_rotation = right.euler_z_rotation(yaw);
    // cout << euler_x_rotation << endl;
    // cout << euler_y_rotation << endl;
    // cout << euler_z_rotation << endl;
    // string seq = "zxy"; // xzy,xyz,yxz,yzx,zyx,zxy
    // string coordinates="right";
    // cout << seq << endl;
    // cout << seq[0]<< endl;
    // cout << seq[1] << endl;
    // cout << seq[2] << endl;
    // Matrix3d dcm = eul2dcm(roll, pitch, yaw, seq, coordinates);
    // cout << dcm << endl;
    // Vector4d quat = eul2quat(roll, pitch, yaw, seq);
    // cout << quat << endl;
    // Matrix3d dcm;
    // dcm(0,0) = 0.000398503;
    // dcm(0,1) = -1;
    // dcm(0,2) = 0.000689382;
    // dcm(1,0) = 0.866158;
    // dcm(1,1) = 0.000689699;
    // dcm(1,2) = 0.49977;
    // dcm(2,0) = -0.49977;
    // dcm(2,1) = 0.000397954;
    // dcm(2,2) = 0.866158;
    // Vector3d eul = dcm2eul(dcm, seq);
    // cout << eul << endl;
    // Vector4d quat = dcm2quat(dcm, seq);
    // cout << quat << endl;

    // Matrix3d dcm;
    // dcm = quat2dcm(0.34, -0.003, -0.892, 0.297);
    // cout << dcm << endl;

    // Vector3d eul = quat2eul(0.34, -0.003, -0.892, 0.297, seq);
    // cout << eul << endl;
    // Vector4d q1(0.34, -0.003, -0.892, 0.297);
    // Vector4d q = quat_conjugate(q1);
    // cout << q << endl;

    // Vector4d q1(0.34, -0.003, -0.892, 0.297);
    // Vector4d q2(0.567, -0.002, -0.781, 0.26);
    // Vector4d q = quat_multi(q1, q2);
    // cout << q << endl;

    // float ax = 0.3;
    // float ay = -0.1;
    // float az = 0.9;
    // float mx = -4;
    // float my = 30;
    // float mz = 13;
    // string nav = "ENU";
    // Vector3d eul;
    // Vector4d quat;
    // eul = acc2eul(ax, ay, az, nav);
    // cout << eul << endl;
    // quat = acc2quat(ax, ay, az, nav);
    // cout << quat << endl;
    // eul = accmag2eul(ax, ay, az, mx, my, mz, nav);
    // cout << eul << endl;
    // quat = accmag2quat(ax, ay, az, mx, my, mz, nav);
    // cout << quat << endl;

    // float ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias, mx_bias, my_bias, mz_bias;
    // float ax_scale, ay_scale, az_scale, gx_scale, gy_scale, gz_scale, mx_scale, my_scale, mz_scale;
    // Node config = LoadFile("../../cfg/config.yaml");
    // ax_bias = config["ENU"]["ax_bias"].as<float>();
    // ay_bias = config["ENU"]["ay_bias"].as<float>();
    // az_bias = config["ENU"]["az_bias"].as<float>();
    // gx_bias = config["ENU"]["gx_bias"].as<float>();
    // gy_bias = config["ENU"]["gy_bias"].as<float>();
    // gz_bias = config["ENU"]["gz_bias"].as<float>();
    // mx_bias = config["ENU"]["mx_bias"].as<float>();
    // my_bias = config["ENU"]["my_bias"].as<float>();
    // mz_bias = config["ENU"]["mz_bias"].as<float>();

    // ax_scale = config["ENU"]["ax_scale"].as<float>();
    // ay_scale = config["ENU"]["ay_scale"].as<float>();
    // az_scale = config["ENU"]["az_scale"].as<float>();
    // gx_scale = config["ENU"]["gx_scale"].as<float>();
    // gy_scale = config["ENU"]["gy_scale"].as<float>();
    // gz_scale = config["ENU"]["gz_scale"].as<float>();
    // mx_scale = config["ENU"]["mx_scale"].as<float>();
    // my_scale = config["ENU"]["my_scale"].as<float>();
    // mz_scale = config["ENU"]["mz_scale"].as<float>();

    // cout << "ax_bias: " << ax_bias << endl;
    // cout << "ay_bias: " << ay_bias << endl;
    // cout << "az_bias: " << az_bias << endl;
    // cout << "gx_bias: " << gx_bias << endl;
    // cout << "gy_bias: " << gy_bias << endl;
    // cout << "gz_bias: " << gz_bias << endl;
    // cout << "mx_bias: " << mx_bias << endl;
    // cout << "my_bias: " << my_bias << endl;
    // cout << "mz_bias: " << mz_bias << endl;

    // cout << "ax_scale: " << ax_scale << endl;
    // cout << "ay_scale: " << ay_scale << endl;
    // cout << "az_scale: " << az_scale << endl;
    // cout << "gx_scale: " << gx_scale << endl;
    // cout << "gy_scale: " << gy_scale << endl;
    // cout << "gz_scale: " << gz_scale << endl;
    // cout << "mx_scale: " << mx_scale << endl;
    // cout << "my_scale: " << my_scale << endl;
    // cout << "mz_scale: " << mz_scale << endl;

    string nav_frame = "ENU";
    int axis = 9;
    float hz = 100;
    bool calibration = false;
    MPU9250 mpu9250(nav_frame, axis, hz, calibration);
    mpu9250.initialization();
    mpu9250.start_thread();
    clock_t start = clock();
    clock_t last = clock();
    while(1)
    {
        start = clock();
        float dt = float(start - last);
        time_sleep(dt, 1/mpu9250.hz);
        cout << "" << endl;
        cout << "temp: " << mpu9250.temp << endl;
        cout << "nav_frame: " << mpu9250.nav_frame << endl;
        cout << "ax ay az: " << mpu9250.ax << " " << mpu9250.ay << " " << mpu9250.az << endl;
        cout << "gx gy gz: " << mpu9250.gx << " " << mpu9250.gy << " " << mpu9250.gz << endl;
        cout << "mx my mz: " << mpu9250.mx << " " << mpu9250.my << " " << mpu9250.mz << endl;
        cout << "roll pitch yaw: " << mpu9250.roll << " " << mpu9250.pitch << " " << mpu9250.yaw << endl;
        cout << "w x y z: " << mpu9250.w << " " << mpu9250.x << " " << mpu9250.y << " " << mpu9250.z << endl;
        last = clock();
        cout << "hz: " << 1/(float(last - start)/CLOCKS_PER_SEC) << endl;
    }
}