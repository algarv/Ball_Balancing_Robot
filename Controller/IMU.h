#ifndef IMU_test__H__
#define IMU_test__H__

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c1
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19 // A DIVIDER FOR SCALING THE GYRO SAMPLE RATE: set to 0 for max
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
#define ACCEL_X 0x3B
#define ACCEL_Y 0x3D
#define ACCEL_Z 0x3F
#define GYRO_X 0x43
#define GYRO_Y 0x45
#define GYRO_Z 0x47
#define ADDR 0x68

struct IMU_Data
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ; 
};

typedef struct IMU_Data IMU_Data;

struct Orientation{
    float roll;
    float pitch;
    float yaw;
};

typedef struct Orientation Orientation;

void IMU_init(void);
Orientation calibrate_IMU(void);
IMU_Data read_IMU();
Orientation calc_RPY(IMU_Data IMU, Orientation calib, double dt);

double pitch_gyro = 0;
double roll_gyro = 0;
double pitch_acc = 0;
double roll_acc = 0;

void IMU_init(void){
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);

    sleep_ms(1000);

    uint8_t reg = 0x1E;
    uint8_t chipID[1];
    uint8_t Ascale = 0;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = 0; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

    i2c_write_blocking(I2C_PORT, ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ADDR, chipID, 1, false);

    uint8_t data[2];
    data[0] = PWR_MGMT_1;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    data[0] = PWR_MGMT_1;
    data[1] = 0x01;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    data[0] = CONFIG;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    data[0] = SMPLRT_DIV;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    int c = i2c_read_blocking(I2C_PORT, ADDR, GYRO_CONFIG, 1, false);

    data[0] = GYRO_CONFIG;
    data[1] = c & ~0xE0;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    data[0] = GYRO_CONFIG;
    data[1] = c & ~0x18;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    data[0] = GYRO_CONFIG;
    data[1] = c | Gscale << 3;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    c = i2c_read_blocking(I2C_PORT, ADDR, ACCEL_CONFIG, 1, false);

    data[0] = ACCEL_CONFIG;
    data[1] = c & ~0xE0;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    data[0] = ACCEL_CONFIG;
    data[1] = c & ~0x18;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);

    data[0] = ACCEL_CONFIG;
    data[1] = c | Ascale << 3;
    i2c_write_blocking(I2C_PORT, ADDR, data, 2, true);
}

Orientation calibrate_IMU(void){
    // IMU_Data IMU_Calibration;
    Orientation RPY_Calibration;

    printf("Calibrating IMU. Place robot on a flat surface.\n");

    // IMU_Data null_calibration;
    Orientation null_calibration;
    null_calibration.roll = 0.0;
    null_calibration.pitch = 0.0;
    null_calibration.pitch = 0.0;

    float accelX[5000], accelY[5000], accelZ[5000], gyroX[5000], gyroY[5000], gyroZ[5000];
    float accelX_avg = 0.0, accelY_avg = 0.0, accelZ_avg = 0.0, gyroX_avg = 0.0, gyroY_avg = 0.0, gyroZ_avg = 0.0;
    float roll_avg = 0.0, pitch_avg = 0.0, yaw_avg = 0.0;

    for(int i=0; i<=5000; i++){
        IMU_Data data = read_IMU();
        Orientation RPY = calc_RPY(data, null_calibration, .250);

        roll_avg += RPY.roll;
        pitch_avg += RPY.pitch;
        yaw_avg += RPY.yaw;
    }

    RPY_Calibration.roll = roll_avg/5000.0;
    RPY_Calibration.pitch = pitch_avg/5000.0;
    RPY_Calibration.yaw = yaw_avg/5000.0;

    return RPY_Calibration;
    // return IMU_Calibration;
}

Orientation calc_RPY(IMU_Data IMU, Orientation calib, double dt){
    Orientation RPY;

    roll_acc = 180 * atan(IMU.accelY/sqrt(IMU.accelX*IMU.accelX + IMU.accelZ*IMU.accelZ))/M_PI; //180*atan2(IMU.accelX,IMU.accelZ)/M_PI;
    pitch_acc = 180 * atan(IMU.accelX/sqrt(IMU.accelY*IMU.accelY + IMU.accelZ*IMU.accelZ))/M_PI; //180*atan2(IMU.accelY,IMU.accelZ)/M_PI;

    roll_gyro = IMU.gyroY * dt;  // Angle around the Y-axis
    pitch_gyro = IMU.gyroX * dt; // Angle around the X-axis

    RPY.roll = 0.7 * (RPY.roll - roll_gyro) + 0.3 * roll_acc;
    RPY.pitch = 0.7 * (RPY.pitch + pitch_gyro) + 0.3 * pitch_acc;
    RPY.yaw = 180 * atan(IMU.accelZ/sqrt(IMU.accelX*IMU.accelX + IMU.accelY*IMU.accelY))/M_PI;

    RPY.roll -= calib.roll;
    RPY.pitch -= calib.pitch;
    RPY.yaw -= calib.yaw;

    return RPY;
}


IMU_Data read_IMU(){

    IMU_Data data;

    uint8_t reg_accel = ACCEL_X;
    uint8_t reg_gyro = GYRO_X;
    uint8_t accel[6];
    uint8_t gyro[6];
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;

    i2c_write_blocking(I2C_PORT, ADDR, &reg_accel, 1, true);
    i2c_read_blocking(I2C_PORT, ADDR, accel, 6, false);

    //(((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    accelX = (((accel[0]<<8) & 0xff00) | (accel[1] & 0x00ff)) & 0xffff;
    accelY = (((accel[2]<<8) & 0xff00) | (accel[3] & 0x00ff)) & 0xffff;
    accelZ = (((accel[4]<<8) & 0xff00) | (accel[5] & 0x00ff)) & 0xffff;

    // Scale to +/- 2g
    data.accelX = (2.0 / 32767.0 * accelX); // - calibration.accelX;
    data.accelY = (2.0 / 32767.0 * accelY); // - calibration.accelY;
    data.accelZ = (2.0 / 32767.0 * accelZ); // - calibration.accelZ;

    i2c_write_blocking(I2C_PORT, ADDR, &reg_gyro, 1, true);
    i2c_read_blocking(I2C_PORT, ADDR, gyro, 6, false);

    gyroX = (((gyro[0]<<8) & 0xff00) | (gyro[1] & 0x00ff)) & 0xffff;
    gyroY = (((gyro[2]<<8) & 0xff00) | (gyro[3] & 0x00ff)) & 0xffff;
    gyroZ = (((gyro[4]<<8) & 0xff00) | (gyro[5] & 0x00ff)) & 0xffff;

    // Scale to +/- 500 deg/s
    data.gyroX = (500.0 / 32767.0 * gyroX); // - calibration.gyroX;
    data.gyroY = (500.0 / 32767.0 * gyroY); // - calibration.gyroY;
    data.gyroZ = (500.0 / 32767.0 * gyroZ); // - calibration.gyroZ;

    return data;
}

#endif