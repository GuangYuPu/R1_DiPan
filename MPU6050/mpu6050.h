#ifndef _MPU_6050_H__
#define _MPU_6050_H__

#include "main.h"

void mpu6050_init();
void mpu6050_decode();

extern uint8_t receiveData[44];
extern int16_t HWT_BIAS;

#endif