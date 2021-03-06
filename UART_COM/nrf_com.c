#include "nrf_com.h"
#include "main.h"
#include "usart.h"

nrfBag_trans_t nrfDataBag;
uint8_t nrfReceive[BAG_LENGTH*2];
int Leftx, Lefty, Rightx, Righty; 
uint8_t button_A,button_B,button_C,button_D,button_E,button_F,button_G,button_H,button_G_last,button_H_last;

//发送单片机初始化
void nrf_Transmit_init()
{
    nrfDataBag.header[0] = HEADER_HIGH;
    nrfDataBag.header[1] = HEADER_MIDDLE_1;
	nrfDataBag.header[2] = HEADER_MIDDLE_2;
	nrfDataBag.header[3] = HEADER_LOW;
}

//发送单片机发送函数，循环调用
void send()
{
    HAL_UART_Transmit(&huart3,nrfDataBag.raw,BAG_LENGTH_TRANS,50);
} 

//接收单片机初始化
void nrf_receive_init()
{
    HAL_UART_Receive_DMA(&huart1,nrfReceive,BAG_LENGTH*2);
}

//在回调函数中进行解码
void nrf_decode()
{
      for(int i = 0 ; i < BAG_LENGTH*2 ; ++i)
      {
          nrfBag_t tempBag = *(nrfBag_t*)(void*)((&nrfReceive[i]));
          if(
              tempBag.header[0] == HEADER_HIGH     &&
              tempBag.header[1] == HEADER_MIDDLE_1 &&
              tempBag.header[2] == HEADER_MIDDLE_2 &&
              tempBag.header[3] == HEADER_LOW  
          ){
              button_G_last = button_G; 
              button_H_last = button_H;
              Leftx = tempBag.Leftx;
              Lefty = tempBag.Lefty;
              Rightx = tempBag.Rightx;
						  Righty = tempBag.Righty;
						button_A = tempBag.button_A;
						button_B = tempBag.button_B;
						button_C = tempBag.button_C;
						button_D = tempBag.button_D;
						button_E = tempBag.button_E;
						button_F = tempBag.button_F;
						button_G = tempBag.button_G;
						button_H = tempBag.button_H;
              break;          
          }
      }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == huart6.Instance)
//    {
//        nrf_decode();
//    }
//}