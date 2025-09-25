/*
 * mpu6050.c
 *
 * Created on: 2025. 04. 15
 *
 * STM32 HAL library for MPU6050 with I2C1
 */
 
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"  // 사용하는 STM32 시리즈에 맞게 조정 (예: stm32f4xx_hal.h)

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    int16_t Temperature;
} MPU6050_t;

/**
 * @brief MPU6050 초기화 함수
 * @param I2Cx 사용 중인 I2C 핸들
 * @param DataStruct 센서 데이터를 저장할 구조체 포인터
 */
void MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

/**
 * @brief MPU6050로부터 모든 센서 데이터 읽기 (가속도, 자이로, 온도)
 * @param I2Cx 사용 중인 I2C 핸들
 * @param DataStruct 데이터를 저장할 구조체 포인터
 */
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

#endif /* MPU6050_H_ */
