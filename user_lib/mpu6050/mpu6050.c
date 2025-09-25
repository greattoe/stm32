/*
 * mpu6050.c
 *
 * Created on: 2025. 04. 15
 *
 * STM32 HAL library for MPU6050 with I2C1
 */

#include "mpu6050.h"

// MPU6050 레지스터 정의
#define MPU6050_ADDR         0x68 << 1 // HAL은 8비트 주소 사용
#define WHO_AM_I_REG         0x75
#define PWR_MGMT_1_REG       0x6B
#define ACCEL_XOUT_H_REG     0x3B

void MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t check;
    uint8_t data;

    // WHO_AM_I 레지스터 확인 (0x68이 반환되어야 함)
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)
    {
        // 장치 웨이크업 (슬립 모드 해제)
        data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
    }
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];

    // 가속도, 온도, 자이로 데이터 한번에 읽기
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    // 가속도 데이터
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 온도 데이터
    DataStruct->Temperature = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

    // 자이로 데이터
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
}


