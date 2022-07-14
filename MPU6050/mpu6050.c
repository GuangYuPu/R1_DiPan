#include "mpu6050.h"
#include "usart.h"

uint8_t receiveData[44];

int16_t HWT_BIAS = 0;

void mpu6050_init()
{
  HAL_UART_Receive_DMA(&huart3,receiveData,44);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	
//}

void mpu6050_decode()
{
	
   for(int i=0;i<44;++i){
       if(receiveData[i] == 0x55    &&
          receiveData[i+1] == 0x52  &&
          receiveData[i+11] == 0x55 &&
          receiveData[i+12] == 0x53 
       )
       {
          HWT_BIAS = ((int16_t)(receiveData[17])) | (((int16_t)(receiveData[18]))<<8);
          break;
		 }
   }
}