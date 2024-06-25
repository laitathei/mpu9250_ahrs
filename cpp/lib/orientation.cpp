#include "orientation.h"

Matrix3d right_hand_rule::euler_x_rotation(float roll)
{
    Matrix3d matrix;
    matrix << 1,0,0,
              0,cos(roll),-sin(roll),
              0,sin(roll),cos(roll);
    return matrix;
}

Matrix3d right_hand_rule::euler_y_rotation(float pitch)
{
    Matrix3d matrix;
    matrix << cos(pitch),0,sin(pitch),
              0,1,0,
              -sin(pitch),0,cos(pitch);
    return matrix;
}

Matrix3d right_hand_rule::euler_z_rotation(float yaw)
{
    Matrix3d matrix;
    matrix << cos(yaw),-sin(yaw),0,
              sin(yaw),cos(yaw),0,
              0,0,1;
    return matrix;
}

Matrix3d left_hand_rule::euler_x_rotation(float roll)
{
    Matrix3d matrix;
    matrix << 1,0,0,
              0,cos(roll),sin(roll),
              0,-sin(roll),cos(roll);
    return matrix;
}

Matrix3d left_hand_rule::euler_y_rotation(float pitch)
{
    Matrix3d matrix;
    matrix << cos(pitch),0,-sin(pitch),
              0,1,0,
              sin(pitch),0,cos(pitch);
    return matrix;
}

Matrix3d left_hand_rule::euler_z_rotation(float yaw)
{
    Matrix3d matrix;
    matrix << cos(yaw),sin(yaw),0,
              -sin(yaw),cos(yaw),0,
              0,0,1;
    return matrix;
}

Vector4d quat_x_rotation(float roll)
{
    Vector4d quaternion;
    float w = cos(roll/2);
    float x = sin(roll/2);
    float y = 0;
    float z = 0;
    quaternion << w, x, y, z;
    return quaternion;
}

Vector4d quat_y_rotation(float pitch)
{
    Vector4d quaternion;
    float w = cos(pitch/2);
    float x = 0;
    float y = sin(pitch/2);
    float z = 0;
    quaternion << w, x, y, z;
    return quaternion;
}

Vector4d quat_z_rotation(float yaw)
{
    Vector4d quaternion;
    float w = cos(yaw/2);
    float x = 0;
    float y = 0;
    float z = sin(yaw/2);
    quaternion << w, x, y, z;
    return quaternion;
}

Matrix3d eul2dcm(float roll, float pitch, float yaw, string seq="xyz", string coordinates="right")
{
    static right_hand_rule right;
    static left_hand_rule left;
    Matrix3d Rx, Ry, Rz;
    if (coordinates == "right"){
        Rx = right.euler_x_rotation(roll);
        Ry = right.euler_y_rotation(pitch);
        Rz = right.euler_z_rotation(yaw);
    }
    else if (coordinates == "left"){
        Rx = left.euler_x_rotation(roll);
        Ry = left.euler_y_rotation(pitch);
        Rz = left.euler_z_rotation(yaw);
    }
    else{
        throw invalid_argument("Only have right or left-handed coordinates system");
    }
    map<char, Matrix3d> R_dict;
    R_dict['x'] = Rx;
    R_dict['y'] = Ry;
    R_dict['z'] = Rz;
    Matrix3d dcm;
    dcm = R_dict.at(seq[0]) * R_dict.at(seq[1]) * R_dict.at(seq[2]);
    return dcm;
}

Vector4d eul2quat(float roll, float pitch, float yaw, string seq)
{
    Vector4d Qx;
    Vector4d Qy;
    Vector4d Qz;
    Qx = quat_x_rotation(roll);
    Qy = quat_y_rotation(pitch);
    Qz = quat_z_rotation(yaw);
    map<char, Vector4d> Q_dict;
    Q_dict['x'] = Qx;
    Q_dict['y'] = Qy;
    Q_dict['z'] = Qz;
    Vector4d quat = quat_multi(quat_multi(Q_dict[seq[0]], Q_dict[seq[1]]), Q_dict[seq[2]]);
    return quat;
}

Vector3d dcm2eul(Matrix3d dcm, string seq)
{
    float det = dcm.determinant();
    if (int(det) != 1)
    {
        throw invalid_argument("Wrong rotation matrix");
    }
    float roll, pitch, yaw;
    if (seq == "xzy"){
        yaw = -asin(dcm(0,1));
        pitch = atan2(dcm(0,2),dcm(0,0));
        roll = atan2(dcm(2,1),dcm(1,1));
    }
    else if (seq == "xyz"){
        yaw = -atan2(dcm(0,1),dcm(0,0));
        pitch = asin(dcm(0,2));
        roll = -atan2(dcm(1,2),dcm(2,2));
    }
    else if (seq == "yxz"){
        yaw = atan2(dcm(1,0),dcm(1,1));
        pitch = atan2(dcm(0,2),dcm(2,2));
        roll = -asin(dcm(1,2));
    }
    else if (seq == "yzx"){
        yaw = asin(dcm(1,0));
        pitch = -atan2(dcm(2,0),dcm(0,0));
        roll = -atan2(dcm(1,2),dcm(1,1));
    }
    else if (seq == "zyx"){
        yaw = atan2(dcm(1,0),dcm(0,0));
        pitch = -asin(dcm(2,0));
        roll = atan2(dcm(2,1),dcm(2,2));
    }
    else if (seq == "zxy"){
        yaw = -atan2(dcm(0,1),dcm(1,1));
        pitch = -atan2(dcm(2,0),dcm(2,2));
        roll =  asin(dcm(2,1));
    }
    Vector3d eul(roll, pitch, yaw);
    return eul;
}

Vector4d dcm2quat(Matrix3d dcm, string seq)
{
    float det = dcm.determinant();
    if (int(det) != 1)
    {
        throw invalid_argument("Wrong rotation matrix");
    }
    float roll, pitch, yaw;
    Vector3d eul;
    Vector4d quat;
    // convert DCM to Euler angle
    eul = dcm2eul(dcm, seq);
    roll = eul[0];
    pitch = eul[1];
    yaw = eul[2];
    // convert Euler angle to Quaternions
    quat = eul2quat(roll, pitch, yaw, seq);
    return quat;
}

Matrix3d quat2dcm(float w, float x, float y, float z)
{
    float n, s;
    float xx, xy, xz, wx, wy, wz, yy, yz, zz;
    Matrix3d dcm;
    n = w*w + x*x + y*y + z*z;
    if (n == 0){
        s = 0;
    }
    else{
        s = 2.0/n;
    }
    xx = x*x; wx = w*x; yy = y*y; 
    xy = x*y; wy = w*y; yz = y*z; 
    xz = x*z; wz = w*z; zz = z*z;
    dcm(0,0) = 1-s*(yy+zz);
    dcm(0,1) = s*(xy-wz);
    dcm(0,2) = s*(xz+wy);
    dcm(1,0) = s*(xy+wz);
    dcm(1,1) = 1-s*(xx+zz);
    dcm(1,2) = s*(yz-wx);
    dcm(2,0) = s*(xz-wy);
    dcm(2,1) = s*(yz+wx);
    dcm(2,2) = 1-s*(xx+yy);
    return dcm;
}

Vector3d quat2eul(float w, float x, float y, float z, string seq)
{
    Matrix3d dcm;
    Vector3d eul;
    float roll, pitch, yaw;
    // convert Quaternion to DCM
    dcm = quat2dcm(w, x, y, z);
    // convert DCM to Euler angle
    eul = dcm2eul(dcm, seq);
    return eul;
}

Vector4d quat_multi(Vector4d q1, Vector4d q2)
{
    float s1, x1, y1, z1, s2, x2, y2, z2;
    s1 = q1[0];
    x1 = q1[1];
    y1 = q1[2];
    z1 = q1[3];
    s2 = q2[0];
    x2 = q2[1];
    y2 = q2[2];
    z2 = q2[3];
    float dot_product;
    Vector3d cross_product;
    dot_product = x1 * x2 + y1 * y2 + z1 * z2;
    cross_product[0] = y1 * z2 - z1 * y2;
    cross_product[1] = -(x1 * z2 - z1 * x2);
    cross_product[2] = x1 * y2 - y1 * x2;
    Vector3d v1(x1, y1, z1), v2(x2, y2, z2);

    float scalar_part = s1 * s2 - dot_product;
    Vector3d vector_part = s1 * v2 + s2 * v1 + cross_product;
    Vector4d q;
    q[0] = scalar_part;
    q[1] = vector_part[0];
    q[2] = vector_part[1];
    q[3] = vector_part[2];
    return q;
}

Vector4d quat_conjugate(Vector4d q)
{
    float w, x, y, z;
    w = q[0];
    x = q[1] * -1;
    y = q[2] * -1;
    z = q[3] * -1;
    Vector4d Q(w, x, y, z);
    return Q;
}