#ifndef _MPU_6050_H__
#define _MPU_6050_H__

#include "main.h"

typedef struct{
    
        int16_t Ax;
        int16_t Ay;
        int16_t Az;
    
    
    
        int16_t Wx;
        int16_t Wy;
        int16_t Wz;
    

    
        int16_t Rx;
        int16_t Ry;
        int16_t Rz;
    
    int16_t T;
    
}MPU6050_t;

void mpu6050_init();
void mpu6050_decode(MPU6050_t* data);

extern uint8_t receiveData[66];
extern MPU6050_t mpu6050_databag;

#endif