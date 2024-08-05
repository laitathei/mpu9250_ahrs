#include "transformation.h"

Vector3d acc2eul(float ax, float ay, float az, string nav)
{
    Vector3d acc, eul;
    float roll, pitch, yaw;
    roll = 0;
    pitch = 0;
    yaw = 0;
    acc << ax, ay, az;
    ax = ax/acc.norm();
    ay = ay/acc.norm();
    az = az/acc.norm();
    if (nav == "ENU"){
        // roll limited between +- 90 degrees as Gimbal Lock problem (Singularity)
        // [ax (E)] = [-sin_y*sin_p*sin_r+cos_y*cos_p   cos_y*sin_p*sin_r+sin_y*cos_p    -sin_p*cos_r][0]
        // [ay (N)] = [-sin_y*cos_r                     cos_y*cos_r                      sin_r       ][0]
        // [az (U)] = [sin_y*cos_p*sin_r+cos_y*sin_p    -cos_y*cos_p*sin_r+sin_y*sin_p   cos_p*cos_r ][g]
        // [ax (E)] = [-gsin_p*cos_r]
        // [ay (N)] = [gsin_r      ]
        // [az (U)] = [gcos_p*cos_r]
        roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))); // or asin(ay)
        pitch = atan2(-ax, az);
        yaw = 0.0;
    }
    else if (nav == "NED"){
        // pitch limited between +- 90 degrees as Gimbal Lock problem (Singularity)
        // [ax (N)] = [cos_y*cos_p                     sin_y*cos_p                     -sin_p     ][0]
        // [ay (E)] = [cos_y*sin_p*sin_r-sin_y*cos_r   sin_y*sin_p*sin_r+cos_y*cos_r   cos_p*sin_r][0]
        // [az (D)] = [cos_y*sin_p*cos_r+sin_y*sin_r   sin_y*sin_p*cos_r-cos_y*sin_r   cos_p*cos_r][g]
        // [ax (N)] = [-gsin_p      ]
        // [ay (E)] = [ gcos_p*sin_r]
        // [az (D)] = [ gcos_p*cos_r]
        roll = atan2(ay, az);
        pitch = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))); // or arcsin(-ax) 
        yaw = 0.0;
    }
    else{
        throw invalid_argument("Navigation frame should be either ENU or NED");
    }
    eul << roll, pitch, yaw;
    return eul;
}

Vector4d acc2quat(float ax, float ay, float az, string nav)
{
    Vector3d eul;
    Vector4d quat;
    float roll, pitch, yaw;
    float w, x, y, z;
    string seq;
    eul = acc2eul(ax, ay, az, nav);
    roll = eul[0];
    pitch = eul[1];
    yaw = eul[2];
    w = 0;
    x = 0;
    y = 0;
    z = 0;
    if (nav == "ENU"){
        // ZXY (yaw - pitch - roll)
        // w = -sin(yaw/2)*sin(pitch/2)*sin(roll/2)+cos(yaw/2)*cos(pitch/2)*cos(roll/2)
        // x = -sin(yaw/2)*sin(pitch/2)*cos(roll/2)+cos(yaw/2)*cos(pitch/2)*sin(roll/2)
        // y = sin(yaw/2)*cos(pitch/2)*sin(roll/2)+cos(yaw/2)*sin(pitch/2)*cos(roll/2)
        // z = sin(yaw/2)*cos(pitch/2)*cos(roll/2)+cos(yaw/2)*sin(pitch/2)*sin(roll/2)
        quat = eul2quat(roll, pitch, yaw, seq="zxy");
        w = quat[0];
        x = quat[1];
        y = quat[2];
        z = quat[3];
    }
    else if (nav == "NED"){
        // w = sin(yaw/2)*sin(pitch/2)*sin(roll/2)+cos(yaw/2)*cos(pitch/2)*cos(roll/2)
        // x = -sin(yaw/2)*sin(pitch/2)*cos(roll/2)+cos(yaw/2)*cos(pitch/2)*sin(roll/2)
        // y = sin(yaw/2)*cos(pitch/2)*sin(roll/2)+cos(yaw/2)*sin(pitch/2)*cos(roll/2)
        // z = sin(yaw/2)*cos(pitch/2)*cos(roll/2)-cos(yaw/2)*sin(pitch/2)*sin(roll/2)
        quat = eul2quat(roll, pitch, yaw, seq="zyx");
        w = quat[0];
        x = quat[1];
        y = quat[2];
        z = quat[3];
    }
    else{
        throw invalid_argument("Navigation frame should be either ENU or NED");
    }
    return quat;
}

Vector3d accmag2eul(float ax, float ay, float az, float mx, float my, float mz, string nav)
{
    Vector3d mag, eul;
    float roll, pitch, yaw;
    eul = acc2eul(ax, ay, az, nav); // accelerometer provide roll and pitch angle
    roll = eul[0];
    pitch = eul[1];
    yaw = eul[2];
    mag << mx, my, mz;
    if (mag.norm() > 0){
        mx = mx/mag.norm();
        my = my/mag.norm();
        mz = mz/mag.norm();
        mag << mx, my, mz;
        if (nav == "ENU"){
            // mx, my, mz in body frame
            // bx, by, bz in navigation frame
            // bx (E) = mx*(-sin_y*sin_p*sin_r+cos_y*cos_p) + my*(-sin_y*cos_r) + mz*(sin_y*cos_p*sin_r+cos_y*sin_p)
            // by (N) = mx*(cos_y*sin_p*sin_r+sin_y*cos_p)  + my*(cos_y*cos_r)  + mz*(-cos_y*cos_p*sin_r+sin_y*sin_p)
            // bz (U) = mx*(-sin_p*cos_r)                   + my*(sin_r)        + mz*(cos_p*cos_r)

            // magnetometer reading in East axis will become 0, when body frame overlap with navigation frame
            // 0 = mx*(-sin_y*sin_p*sin_r+cos_y*cos_p) + my*(-sin_y*cos_r) + mz*(sin_y*cos_p*sin_r+cos_y*sin_p)
            yaw = atan2(mx*cos(pitch) + mz*sin(pitch), mx*sin(pitch)*sin(roll) + my*cos(roll) - mz*cos(pitch)*sin(roll));
        }
        else if (nav == "NED"){
            // mx, my, mz in body frame
            // bx, by, bz in navigation frame
            // bx (N) = mx*(cos_y*cos_p) + my*(cos_y*sin_p*sin_r-sin_y*cos_r)  + mz*(cos_y*sin_p*cos_r+sin_y*sin_r)
            // by (E) = mx*(sin_y*cos_p) + my*(sin_y*sin_p*sin_r+cos_y*cos_r)  + mz*(sin_y*sin_p*cos_r-cos_y*sin_r)
            // bz (D) = mx*(-sin_p)      + my*(cos_p*sin_r)                    + mz*(cos_p*cos_r)

            // magnetometer reading in East axis will become 0, when body frame overlap with navigation frame
            // 0 = mx*(sin_y*cos_p) + my*(sin_y*sin_p*sin_r+cos_y*cos_r)  + mz*(sin_y*sin_p*cos_r-cos_y*sin_r)
            yaw = -atan2(my*cos(roll) - mz*sin(roll), mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll));
        }
        else{
            throw invalid_argument("Navigation frame should be either ENU or NED");
        }
    }
    eul << roll, pitch, yaw;
    return eul;
}

Vector4d accmag2quat(float ax, float ay, float az, float mx, float my, float mz, string nav)
{
    Vector3d eul;
    Vector4d quat;
    float roll, pitch, yaw;
    float w, x, y, z;
    string seq;
    eul = accmag2eul(ax, ay, az, mx, my, mz, nav);
    roll = eul[0];
    pitch = eul[1];
    yaw = eul[2];
    w = 0;
    x = 0;
    y = 0;
    z = 0;
    if (nav == "ENU")
    {
        // w = -sin(yaw/2)*sin(pitch/2)*sin(roll/2)+cos(yaw/2)*cos(pitch/2)*cos(roll/2)
        // x = -sin(yaw/2)*sin(pitch/2)*cos(roll/2)+cos(yaw/2)*cos(pitch/2)*sin(roll/2)
        // y = sin(yaw/2)*cos(pitch/2)*sin(roll/2)+cos(yaw/2)*sin(pitch/2)*cos(roll/2)
        // z = sin(yaw/2)*cos(pitch/2)*cos(roll/2)+cos(yaw/2)*sin(pitch/2)*sin(roll/2)
        quat = eul2quat(roll, pitch, yaw, seq="zxy");
        w = quat[0];
        x = quat[1];
        y = quat[2];
        z = quat[3];
    }
    else if (nav == "NED"){
        // w = sin(yaw/2)*sin(pitch/2)*sin(roll/2)+cos(yaw/2)*cos(pitch/2)*cos(roll/2)
        // x = -sin(yaw/2)*sin(pitch/2)*cos(roll/2)+cos(yaw/2)*cos(pitch/2)*sin(roll/2)
        // y = sin(yaw/2)*cos(pitch/2)*sin(roll/2)+cos(yaw/2)*sin(pitch/2)*cos(roll/2)
        // z = sin(yaw/2)*cos(pitch/2)*cos(roll/2)-cos(yaw/2)*sin(pitch/2)*sin(roll/2)
        quat = eul2quat(roll, pitch, yaw, seq="zyx");
        w = quat[0];
        x = quat[1];
        y = quat[2];
        z = quat[3];
    }
    else{
        throw invalid_argument("Navigation frame should be either ENU or NED");
    }
    return quat;
}

Vector3d ENU2NED(float E, float N, float U)
{
    Vector3d ENU, NED;
    ENU << E, N, U;
    Matrix3d matrix;
    matrix << 0, 1, 0, 
              1, 0, 0,
              0, 0, -1;
    NED = matrix * ENU;
    return NED;
}

Vector3d NED2ENU(float N, float E, float D)
{
    Vector3d ENU, NED;
    NED << N, E, D;
    Matrix3d matrix;
    matrix << 0, 1, 0, 
              1, 0, 0,
              0, 0, -1;
    ENU = matrix * NED;
    return ENU;
}

Matrix3d skew_symmetric(float x, float y, float z)
{
    Matrix3d matrix;
    matrix << 0, -z, y, 
              z, 0, -x,
              -y, x, 0.0;
    return matrix;
}