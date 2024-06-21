#include <math.h>
#include <sstream>
#include "mpu6500.h"
#include "transformation.h"
#include "utils.h"

void MPU6500::control_accel_gyro(bool ax=true, bool ay=true, bool az=true, bool gx=true, bool gy=true, bool gz=true)
{
    int value = 0; // 0b00000000
    if (ax == false){
        value += 4; // 0b10000000
    }
    else if (ay == false){
        value += 8; // 0b01000000
    }
    else if (az == false){
        value += 16; // 0b00100000
    }
    else if (gx == false){
        value += 32; // 0b10000000
    }
    else if (gy == false){
        value += 64; // 0b01000000
    }
    else if (gz == false){
        value += 128; // 0b00100000
    }
    wiringPiI2CWriteReg8(this->i2c, PWR_MGMT_2, value);
}

void MPU6500::who_am_i()
{
    int value;
    value = this->read_8bit_register(WHO_AM_I);
    cout << "The register value is 0x" << hex << value << endl;
    if (value == 0x71){
        cout << "It is MPU6500 default value" << endl;
    }
    else{
        cout << "It is not MPU6500 default value" << endl;
        throw invalid_argument("MPU6500 not found");
    }
}

void MPU6500::config_MPU6500(int accel_parameter, int gyro_parameter)
{
    // config accelerometer full scale
    if (accel_parameter == ACCEL_FS_SEL_2G){
        this->accel_fs = 2.0/32768.0;
    }
    else if (accel_parameter == ACCEL_FS_SEL_4G){
        this->accel_fs = 4.0/32768.0;
    }
    else if (accel_parameter == ACCEL_FS_SEL_8G){
        this->accel_fs = 8.0/32768.0;
    }
    else if (accel_parameter == ACCEL_FS_SEL_16G){
        this->accel_fs = 16.0/32768.0;
    }
    else{
        throw invalid_argument("Wrong accel config parameter");
    }

    // config gyroscope full scale
    if (gyro_parameter == GYRO_FS_SEL_250DPS){
        this->gyro_fs = 250.0/32768.0;
    }
    else if (gyro_parameter == GYRO_FS_SEL_500DPS){
        this->gyro_fs = 500.0/32768.0;
    }
    else if (gyro_parameter == GYRO_FS_SEL_1000DPS){
        this->gyro_fs = 1000.0/32768.0;
    }
    else if (gyro_parameter == GYRO_FS_SEL_2000DPS){
        this->gyro_fs = 2000.0/32768.0;
    }
    else{
        throw invalid_argument("Wrong gyro config parameter");
    }

    // Write byte data to MPU6500 gyroscope and accelerometer configuration register
    wiringPiI2CWriteReg8(this->i2c, ACCEL_CONFIG, accel_parameter);
    wiringPiI2CWriteReg8(this->i2c, GYRO_CONFIG, gyro_parameter);

    // MPU6500 and AK8963 share the same I2C
    // MPU6500 is master
    // AK8963 is slave
    int value = 0x02;
    wiringPiI2CWriteReg8(this->i2c, BYPASS_ENABLE, value);

    // ACCEL_CONFIG_2   | Bandwidth | Delay   
    // 0x00             | 250Hz     | 1.88ms
    // 0x01             | 184Hz     | 1.88ms  
    // 0x02             | 92Hz      | 2.88ms  
    // 0x03             | 41Hz      | 4.88ms  
    // 0x04             | 20Hz      | 8.87ms  
    // 0x05             | 10Hz      | 16.83ms
    // 0x06             | 5Hz       | 32.48ms
    // 0x07             | 3600Hz    | 1.38ms

    // GYRO_CONFIG_2   | Bandwidth | Delay   | Fs   | 
    // 0x00            | 250Hz     | 0.97ms  | 8kHz | 
    // 0x01            | 184Hz     | 2.9ms   | 1kHz | 
    // 0x02            | 92Hz      | 3.9ms   | 1kHz | 
    // 0x03            | 41Hz      | 5.9ms   | 1kHz | 
    // 0x04            | 20Hz      | 9.9ms   | 1kHz | 
    // 0x05            | 10Hz      | 17.85ms | 1kHz | 
    // 0x06            | 5Hz       | 33.48ms | 1kHz | 
    // 0x07            | 3600Hz    | 0.17ms  | 8kHz | 
    wiringPiI2CWriteReg8(this->i2c, ACCEL_CONFIG_2, 0x06); // Set accel digital high-pass filter
    wiringPiI2CWriteReg8(this->i2c, GYRO_CONFIG_2, 0x06); // Set gyro digital low-pass filter
    wiringPiI2CWriteReg8(this->i2c, SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
}

int MPU6500::read_raw_data(int high_register, int low_register)
{
    int high=-1, low=-1, unsigned_value, signed_value;
    while (high == -1) {
        high = wiringPiI2CReadReg8(this->i2c, high_register);
        if (high != -1) {
            break;
        }
    }
    while (low == -1) {
        low = wiringPiI2CReadReg8(this->i2c, low_register);
        if (low != -1) {
            break;
        }
    }

    // Megre higher bytes and lower bytes data
    unsigned_value = (high << 8) + low;

    // Calculate the unsigned int16 range to signed int16 range
    if ((unsigned_value >= 32768) and (unsigned_value < 65536)){
        signed_value = -(~(unsigned_value - 65536)+1);
    }
    else if ((unsigned_value >= 0) and (unsigned_value < 32768)){
        signed_value = unsigned_value;
    }
    return signed_value;
}

int MPU6500::read_8bit_register(int single_register)
{
    int data=-1;
    while (data == -1) {
        data = wiringPiI2CReadReg8(this->i2c, single_register);
        if (data != -1) {
            break;
            return data;
        }
    }
}

accel_gyro_calib MPU6500::gyro_calibration(float s)
{
    if ((s > 0) and (this->calibration == true))
    {
        stringstream ss;
        Vector3d total_gyro, current_gyro, avg_gyro;
        total_gyro << 0, 0, 0;
        ss << "Start gyroscope calibration - Do not move the IMU for " << s << "s";
        cout << ss.str();
        cin.ignore(); // Press enter with nothing

        for (int i=0;i<(s*this->hz);i++)
        {
            current_gyro = this->get_gyro();
            total_gyro = total_gyro + current_gyro;
            time_sleep(0, 1/this->hz);
        }

        avg_gyro = total_gyro/(s*this->hz);
        this->gx_bias = avg_gyro[0];
        this->gy_bias = avg_gyro[1];
        this->gz_bias = avg_gyro[2];

        // calculate bias
        this->gyro_bias << this->gx_bias, this->gy_bias, this->gz_bias;

        // calculate scale
        this->gyro_scale << this->gx_scale, this->gy_scale, this->gz_scale;

        // calculate misalignment
        this->gyro_misalignment << this->gxy_mis, this->gxz_mis, this->gyx_mis, this->gyz_mis, this->gzx_mis, this->gzy_mis;
    }
    accel_gyro_calib result;
    result.scale = this->gyro_scale;
    result.bias = this->gyro_bias;
    result.misalignment = this->gyro_misalignment;
    cout << "Finish gyroscope calibration" << endl;
    return result;
}

accel_gyro_calib MPU6500::accel_calibration(float s)
{
    MatrixXd target(6,3);
    MatrixXd error_matrix(4,3);
    float x_bias, y_bias, z_bias;
    float x_scale, y_scale, z_scale;
    float xy_mis, xz_mis, yx_mis, yz_mis, zx_mis, zy_mis;
    if ((s > 0) and (this->calibration == true))
    {
        const char* order[6] = {"x","y","z","-x","-y","-z"};
        MatrixXd calibration(6,3);
        VectorXd negative_one(6,1);
        negative_one << -1, -1, -1, -1, -1, -1;
        for (int i=0;i<6;i++)
        {
            stringstream ss;
            Vector3d total_accel, current_accel, avg_accel;
            total_accel << 0, 0, 0;
            ss << "Place IMU " << order[i] << " axis (" << this->nav_frame << ") pointing downward and do not move the IMU for " << s << "s";
            cout << ss.str();
            cin.ignore(); // Press enter with nothing
            
            for (int j=0;j<(s*this->hz);j++)
            {
                current_accel = this->get_accel();
                total_accel = total_accel + current_accel;
                time_sleep(0, 1/this->hz);
            }
            avg_accel = total_accel/(s*this->hz);
            calibration.row(i) = avg_accel.transpose();
        }
        calibration.conservativeResize(calibration.rows(), calibration.cols()+1); // append one columns
        calibration.col(calibration.cols()-1) = negative_one; // replace last columns to negative one
        Matrix<double, 3, 1> positive = Matrix<double, 3, 1>::Constant(g);
        Matrix<double, 3, 1> negative = Matrix<double, 3, 1>::Constant(-g);
        Matrix<double, 3, 3> positive_matrix = positive.asDiagonal();
        Matrix<double, 3, 3> negative_matrix = negative.asDiagonal();

        if (this->nav_frame == "ENU")
        {
            target << negative_matrix, positive_matrix;
        }
        else if (this->nav_frame == "NED")
        {
            target << positive_matrix, negative_matrix;
        }

        error_matrix = (calibration.transpose() * calibration);
        error_matrix = error_matrix.inverse().eval();
        error_matrix = error_matrix * calibration.transpose() * target;

        // calculate bias
        this->ax_bias = error_matrix(3,0);
        this->ay_bias = error_matrix(3,1);
        this->az_bias = error_matrix(3,2);
        this->accel_bias << this->ax_bias, this->ay_bias, this->az_bias;

        // calculate scale
        this->ax_scale = error_matrix(0,0);
        this->ay_scale = error_matrix(1,1);
        this->az_scale = error_matrix(2,2);
        this->accel_scale << this->ax_scale, this->ay_scale, this->az_scale;

        // calculate misalignment
        this->axy_mis = error_matrix(0,1);
        this->axz_mis = error_matrix(0,2);
        this->ayx_mis = error_matrix(1,0);
        this->ayz_mis = error_matrix(1,2);
        this->azx_mis = error_matrix(2,0);
        this->azy_mis = error_matrix(2,1);
        this->accel_misalignment << this->axy_mis, this->axz_mis, this->ayx_mis, this->ayz_mis, this->azx_mis, this->azy_mis;
    }
    accel_gyro_calib result;
    result.scale = this->accel_scale;
    result.bias = this->accel_bias;
    result.misalignment = this->accel_misalignment;
    cout << "Finish accelerometer calibration" << endl;
    return result;
}

Vector3d MPU6500::get_accel()
{
    Vector3d accel;
    float ax, ay, az;
    ax = this->read_raw_data(ACCEL_XOUT_H, ACCEL_XOUT_L)*this->accel_fs;
    ay = this->read_raw_data(ACCEL_YOUT_H, ACCEL_YOUT_L)*this->accel_fs;
    az = this->read_raw_data(ACCEL_ZOUT_H, ACCEL_ZOUT_L)*this->accel_fs;

    // convert to m/s^2
    ax = ax*g;
    ay = ay*g;
    az = az*g;

    // convert to NED frame
    if (this->nav_frame == "NED"){
        ax = ax*-1;
        ay = ay*-1;
        az = az*-1;
        accel = ENU2NED(ax, ay, az);
        ax = accel[0];
        ay = accel[1];
        az = accel[2];
    }

    // accelerometer model: calibrated measurement = (matrix)*(raw measurement - bias)
    if (this->calibration == false)
    {
        ax = (this->ax_scale * ax + this->axy_mis * ay + this->axz_mis * az) - this->ax_bias;
        ay = (this->ayx_mis * ax + this->ay_scale * ay + this->ayz_mis * az) - this->ay_bias;
        az = (this->azx_mis * ax + this->azy_mis * ay + this->az_scale * az) - this->az_bias;        
    }

    accel << ax, ay, az;
    return accel;
}

Vector3d MPU6500::get_gyro()
{
    Vector3d gyro;
    float gx, gy, gz;
    gx = this->read_raw_data(GYRO_XOUT_H, GYRO_XOUT_L)*this->gyro_fs*M_PI/180;
    gy = this->read_raw_data(GYRO_YOUT_H, GYRO_YOUT_L)*this->gyro_fs*M_PI/180;
    gz = this->read_raw_data(GYRO_ZOUT_H, GYRO_ZOUT_L)*this->gyro_fs*M_PI/180;

    // convert to NED frame
    if (this->nav_frame == "NED"){
        gyro = ENU2NED(gx, gy, gz);
        gx = gyro[0];
        gy = gyro[1];
        gz = gyro[2];
    }

    // gyroscope model: calibrated measurement = (matrix)*(raw measurement - bias)
    if (this->calibration == false)
    {
        gx = (this->gx_scale * gx + this->gxy_mis * gy + this->gxz_mis * gz) - this->gx_bias;
        gy = (this->gyx_mis * gx + this->gy_scale * gy + this->gyz_mis * gz) - this->gy_bias;
        gz = (this->gzx_mis * gx + this->gzy_mis * gy + this->gz_scale * gz) - this->gz_bias;
    }
    
    gyro << gx, gy, gz;
    return gyro;
}

float MPU6500::get_temp()
{
    float temp = this->read_raw_data(TEMP_OUT_H, TEMP_OUT_L);
    temp = ((temp - ROOM_TEMP_OFFSET)/TEMP_SENSITIVITY) + 21.0;
    return temp;
}