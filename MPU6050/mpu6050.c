#include "mpu6050.h"
#include "usart.h"

uint8_t receiveData[66];

MPU6050_t mpu6050_databag;

void mpu6050_init()
{
  HAL_UART_Receive_DMA(&huart3,receiveData,66);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	
//}

void mpu6050_decode(MPU6050_t* data)
{
   for(int i=0;i<66;++i){
       if(receiveData[i] == 0x55    &&
          receiveData[i+1] == 0x51  &&
          receiveData[i+11] == 0x55 &&
          receiveData[i+12] == 0x52 &&
          receiveData[i+22] == 0x55 &&
          receiveData[i+23] == 0x53
       )
       {
         data->Ax = ((float)(((int16_t)(receiveData[i+3]<<8))|((int16_t)(receiveData[i+2])))/32768)*16*(9.8);
				 data->Ay = ((float)(((int16_t)(receiveData[i+5]<<8))|((int16_t)(receiveData[i+4])))/32768)*16*(9.8);
				 data->Az = ((float)(((int16_t)(receiveData[i+7]<<8))|((int16_t)(receiveData[i+6])))/32768)*16*(9.8);
				 
				 data->Wx = ((float)(((int16_t)(receiveData[i+14]<<8))|((int16_t)(receiveData[i+13])))/32768)*2000;
				 data->Wy = ((float)(((int16_t)(receiveData[i+16]<<8))|((int16_t)(receiveData[i+15])))/32768)*2000;
				 data->Wz = ((float)(((int16_t)(receiveData[i+18]<<8))|((int16_t)(receiveData[i+17])))/32768)*2000;
  
				 data->Rx = ((float)(((int16_t)(receiveData[i+25]<<8))|((int16_t)(receiveData[i+24])))/32768)*180;
				 data->Ry = ((float)(((int16_t)(receiveData[i+27]<<8))|((int16_t)(receiveData[i+26])))/32768)*180;
				 data->Rz = ((float)(((int16_t)(receiveData[i+29]<<8))|((int16_t)(receiveData[i+28])))/32768)*180;
				 
				 data->T =  ((float)((receiveData[i+31]<<8)|receiveData[i+30])/340) + 37;
			 }
   }
}