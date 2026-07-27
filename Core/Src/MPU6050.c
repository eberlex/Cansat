#include "mpu6050.h"

/*================== REGISTRADORES INTERNOS ==================*/

#define WHO_AM_I_REG           0x75
#define PWR_MGMT1_REG          0x6B
#define SMPLRT_DIV_REG         0x19
#define CONFIG_REG             0x1A
#define GYRO_CONFIG_REG        0x1B
#define ACCEL_CONFIG_REG       0x1C

#define ACCEL_XOUT_H_REG       0x3B
#define TEMP_OUT_H_REG         0x41
#define GYRO_XOUT_H_REG        0x43

/*================== FUNÇÕES AUXILIARES I2C ==================*/

static HAL_StatusTypeDef MPU6050_Write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(
                hi2c,
                MPU6050_I2C_ADDR,
                reg,
                I2C_MEMADD_SIZE_8BIT,
                &data,
                1,
                HAL_MAX_DELAY);
}

static HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, uint16_t length)
{
    return HAL_I2C_Mem_Read(
                hi2c,
                MPU6050_I2C_ADDR,
                reg,
                I2C_MEMADD_SIZE_8BIT,
                buffer,
                length,
                HAL_MAX_DELAY);
}

/*===========================================================
 *      INICIALIZAÇÃO
 *==========================================================*/

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;

    /* Verifica comunicação leitor de ID */
    if(MPU6050_Read(hi2c, WHO_AM_I_REG, &check, 1) != HAL_OK)
        return HAL_ERROR;

    if(check != 0x68)
        return HAL_ERROR;

    /* Sai do modo Sleep e usa o PLL com referência no giroscópio de eixo X */
    if(MPU6050_Write(hi2c, PWR_MGMT1_REG, 0x01) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(100);

    /* Sample Rate = 1kHz (DLPF habilitado) */
    if(MPU6050_Write(hi2c, SMPLRT_DIV_REG, 0x00) != HAL_OK)
        return HAL_ERROR;

    /* DLPF = ~42Hz */
    if(MPU6050_Write(hi2c, CONFIG_REG, 0x03) != HAL_OK)
        return HAL_ERROR;

    /* Gyro Config ±250°/s */
    if(MPU6050_Write(hi2c, GYRO_CONFIG_REG, 0x00) != HAL_OK)
        return HAL_ERROR;

    /* Accel Config ±2g */
    if(MPU6050_Write(hi2c, ACCEL_CONFIG_REG, 0x00) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/*==========================================================
 *      LEITURAS DE DADOS
 *==========================================================*/

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *imu)
{
    uint8_t data[14];

    if(MPU6050_Read(hi2c, ACCEL_XOUT_H_REG, data, 14) != HAL_OK)
        return HAL_ERROR;

    imu->Accel_X_RAW     = (int16_t)(((uint16_t)data[0]  << 8) | data[1]);
    imu->Accel_Y_RAW     = (int16_t)(((uint16_t)data[2]  << 8) | data[3]);
    imu->Accel_Z_RAW     = (int16_t)(((uint16_t)data[4]  << 8) | data[5]);

    imu->Temperature_RAW = (int16_t)(((uint16_t)data[6]  << 8) | data[7]);

    imu->Gyro_X_RAW      = (int16_t)(((uint16_t)data[8]  << 8) | data[9]);
    imu->Gyro_Y_RAW      = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
    imu->Gyro_Z_RAW      = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, MPU6050_t *imu)
{
    uint8_t data[6];

    if(MPU6050_Read(hi2c, ACCEL_XOUT_H_REG, data, 6) != HAL_OK)
        return HAL_ERROR;

    imu->Accel_X_RAW = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    imu->Accel_Y_RAW = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    imu->Accel_Z_RAW = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, MPU6050_t *imu)
{
    uint8_t data[6];

    if(MPU6050_Read(hi2c, GYRO_XOUT_H_REG, data, 6) != HAL_OK)
        return HAL_ERROR;

    imu->Gyro_X_RAW = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    imu->Gyro_Y_RAW = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    imu->Gyro_Z_RAW = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadTemperature(I2C_HandleTypeDef *hi2c, MPU6050_t *imu)
{
    uint8_t data[2];

    if(MPU6050_Read(hi2c, TEMP_OUT_H_REG, data, 2) != HAL_OK)
        return HAL_ERROR;

    imu->Temperature_RAW = (int16_t)(((uint16_t)data[0] << 8) | data[1]);

    return HAL_OK;
}

/*==========================================================
 *      CONVERSÃO PARA UNIDADES FÍSICAS
 *==========================================================*/

void MPU6050_Convert(MPU6050_t *imu)
{
    /* Converte aceleração RAW para 'g' (considerando escala ±2g) */
    imu->Ax = (float)imu->Accel_X_RAW / MPU6050_ACCEL_SENS_2G;
    imu->Ay = (float)imu->Accel_Y_RAW / MPU6050_ACCEL_SENS_2G;
    imu->Az = (float)imu->Accel_Z_RAW / MPU6050_ACCEL_SENS_2G;

    /* Converte giroscópio RAW para °/s (considerando escala ±250°/s) */
    imu->Gx = (float)imu->Gyro_X_RAW / MPU6050_GYRO_SENS_250;
    imu->Gy = (float)imu->Gyro_Y_RAW / MPU6050_GYRO_SENS_250;
    imu->Gz = (float)imu->Gyro_Z_RAW / MPU6050_GYRO_SENS_250;

    /* Converte temperatura RAW para °C conforme Datasheet MPU6050 */
    imu->Temperature = ((float)imu->Temperature_RAW / 340.0f) + 36.53f;
}