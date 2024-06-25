#pragma once
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <map>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class right_hand_rule
{
    /**
     * @brief Right-handed coordinates system rotation
     */
    public:
        Matrix3d euler_x_rotation(float roll);
        Matrix3d euler_y_rotation(float pitch);
        Matrix3d euler_z_rotation(float yaw);
};

class left_hand_rule
{
    /**
     * @brief Left-handed coordinates system rotation
     */
    public:
        Matrix3d euler_x_rotation(float roll);
        Matrix3d euler_y_rotation(float pitch);
        Matrix3d euler_z_rotation(float yaw);
};

Vector4d quat_x_rotation(float roll);
Vector4d quat_y_rotation(float pitch);
Vector4d quat_z_rotation(float yaw);

Matrix3d eul2dcm(float roll, float pitch, float yaw, string seq, string coordinates);
Vector4d eul2quat(float roll, float pitch, float yaw, string seq);

Vector3d dcm2eul(Matrix3d dcm, string seq);
Vector4d dcm2quat(Matrix3d dcm, string seq);

Matrix3d quat2dcm(float w, float x, float y, float z);
Vector3d quat2eul(float w, float x, float y, float z, string seq);

Vector4d quat_multi(Vector4d q1, Vector4d q2);
Vector4d quat_conjugate(Vector4d q);