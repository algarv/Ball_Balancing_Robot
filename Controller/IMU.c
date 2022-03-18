#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "IMU_test.h"

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

IMU_Data calibrate_IMU(void){

    printf("Calibrating IMU. Place robot on a flat surface.\n");

    IMU_Data IMU_Calibration;

    float accelX[1000], accelY[1000], accelZ[1000], gyroX[1000], gyroY[1000], gyroZ[1000];
    float accelX_avg = 0, accelY_avg = 0, accelZ_avg = 0, gyroX_avg = 0, gyroY_avg = 0, gyroZ_avg = 0;
    
    for(int i=0; i<=1000; i++){
        IMU_Data data = read_IMU();
        accelX_avg += data.accelX;
        accelY_avg += data.accelY;
        accelZ_avg += data.accelZ;
        gyroX_avg += data.gyroX;
        gyroY_avg += data.gyroY;
        gyroZ_avg += data.gyroZ;
    }

    IMU_Calibration.accelX = accelX_avg/1000;
    IMU_Calibration.accelY = accelY_avg/1000;
    IMU_Calibration.accelZ = accelZ_avg/1000;
    IMU_Calibration.gyroX = gyroX_avg/1000;
    IMU_Calibration.gyroY = gyroY_avg/1000;
    IMU_Calibration.gyroZ = gyroZ_avg/1000;

    return IMU_Calibration;
}

IMU_Data read_IMU(void){

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
    data.accelX = (((accel[0]<<8) & 0xff00) | (accel[1] & 0x00ff)) & 0xffff;
    data.accelY = (((accel[2]<<8) & 0xff00) | (accel[3] & 0x00ff)) & 0xffff;
    data.accelZ = (((accel[4]<<8) & 0xff00) | (accel[5] & 0x00ff)) & 0xffff;

    // Scale to +/- 2g
    data.accelX = 2.0 / 32767.0 * accelX;
    data.accelY = 2.0 / 32767.0 * accelY;
    data.accelZ = 2.0 / 32767.0 * accelZ;

    i2c_write_blocking(I2C_PORT, ADDR, &reg_gyro, 1, true);
    i2c_read_blocking(I2C_PORT, ADDR, gyro, 6, false);

    data.gyroX = (((gyro[0]<<8) & 0xff00) | (gyro[1] & 0x00ff)) & 0xffff;
    data.gyroY = (((gyro[2]<<8) & 0xff00) | (gyro[3] & 0x00ff)) & 0xffff;
    data.gyroZ = (((gyro[4]<<8) & 0xff00) | (gyro[5] & 0x00ff)) & 0xffff;

    // Scale to +/- 500 deg/s
    data.gyroX = 500.0 / 32767.0 * gyroX;
    data.gyroY = 500.0 / 32767.0 * gyroY;
    data.gyroZ = 500.0 / 32767.0 * gyroZ;

    // Print to serial monitor
    printf("Accelerometer\n");
    printf("X: %6.2f Y: %6.2f Z: %6.2f\n", data.accelX, data.accelY, data.accelZ);
    printf("Gyroscope\n");
    printf("X: %6.2f Y: %6.2f Z: %6.2f\n", data.gyroX, data.gyroY, data.gyroZ);
    sleep_ms(500);

    return data;
}