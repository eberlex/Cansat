#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================
 *
 *                      Includes
 *
 *==========================================================*/

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*==========================================================
 *
 *                  Endereço I2C
 *
 *==========================================================*/

/*
 * AD0 = GND -> 0x68
 * AD0 = VCC -> 0x69
 */

#define MPU6050_I2C_ADDR        (0x68 << 1)

/*==========================================================
 *
 *              Sensibilidade do acelerômetro
 *
 *==========================================================*/

#define MPU6050_ACCEL_SENS_2G      16384.0f
#define MPU6050_ACCEL_SENS_4G       8192.0f
#define MPU6050_ACCEL_SENS_8G       4096.0f
#define MPU6050_ACCEL_SENS_16G      2048.0f

/*==========================================================
 *
 *              Sensibilidade do giroscópio
 *
 *==========================================================*/

#define MPU6050_GYRO_SENS_250      131.0f
#define MPU6050_GYRO_SENS_500       65.5f
#define MPU6050_GYRO_SENS_1000      32.8f
#define MPU6050_GYRO_SENS_2000      16.4f

/*==========================================================
 *
 *              Estrutura principal
 *
 *==========================================================*/

typedef struct
{
    /*---------------- RAW ----------------*/

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    int16_t Temperature_RAW;

    /*----------- Valores convertidos -----------*/

    float Ax;
    float Ay;
    float Az;

    float Gx;
    float Gy;
    float Gz;

    float Temperature;

} MPU6050_t;

/*==========================================================
 *
 *                  Protótipos
 *
 *==========================================================*/

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c,
                                  MPU6050_t *imu);

HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,
                                    MPU6050_t *imu);

HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,
                                   MPU6050_t *imu);

HAL_StatusTypeDef MPU6050_ReadTemperature(I2C_HandleTypeDef *hi2c,
                                          MPU6050_t *imu);

void MPU6050_Convert(MPU6050_t *imu);

#ifdef __cplusplus
}
#endif

#endif