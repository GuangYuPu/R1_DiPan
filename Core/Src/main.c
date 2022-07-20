/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "omni.h"
#include "nrf_com.h"
#include "Caculate.h"
#include "DJI.h"
#include "wtr_can.h"
#include "ADS1256.h"
#include "mpu6050.h"
#include "Wtr_MotionPlan.h"
#include "main.h"
#include "stdio.h"  
#include "wtr_mavlink.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int ref_x = 0;
int ref_y = 0;
int cur_x = 0;
int cur_y = 0;
int cur_gate[4] = {0,0,0,0};
int gate_y = 0;
int Ref_x[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
int Ref_y[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};

int ifRecv = 0;
int ifRecv_mpu = 0;
int16_t HWT_init = 0;
int16_t HWT_init_lsat = 0;

uint32_t time = 0;
uint32_t enter_time = 0;
uint32_t tick = 0;

uint32_t counter = 0;

int b = 0;
int a = 0;

int state = 0;
int index_r = 0;
int index_b = 0;

uint8_t region = 0;

uint8_t zone = 0;
uint8_t qu_qiu = 0;
uint8_t she_qiu = 0;

float robot_vx = 0;
float robot_vy = 0;
float robot_rot = 0;
float robot_vx_last = 0;
float robot_vy_last = 0;
float robot_rot_last = 0;
float k_bias = 0.15;

int32_t Bias_mpu = 0;

mavlink_controller_t ControllerData = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	
	CANFilterInit(&hcan1);
	
	hDJI[0].motorType = M3508;
	hDJI[1].motorType = M3508;
	hDJI[2].motorType = M3508;
	hDJI[3].motorType = M3508;
	
	DJI_Init();
	
	Kine_Init(0.55,0.55,0,0);
	
	ifRecv = 0;
	
	ADS1256_Init();
  // MotionPlan_Init(((float)(ADS1256_diff_data[3]))/547098.f,((float)(ADS1256_diff_data[0]))/547098.f);
	
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	
	nrf_Transmit_init();
	// nrf_receive_init();

  WTR_MAVLink_Init(&huart1,MAVLINK_COMM_0);
  WTR_MAVLink_RcvStart(MAVLINK_COMM_0);

	mpu6050_init();
	HWT_init = HWT_BIAS;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    tick = HAL_GetTick();
		while(!ifRecv)
		{
      time = 0;
		}
		
    ADS1256_UpdateDiffData();
    
    Leftx = ControllerData.left_x + 2048;
    Rightx = ControllerData.right_x + 2048;
    Lefty = ControllerData.left_y + 2048;
    Righty = ControllerData.right_y + 2048;
    button_A = ControllerData.buttons & (1<<1);
    button_B = ControllerData.buttons & (1<<1);
    button_C = ControllerData.buttons & (1<<1);
    button_D = ControllerData.buttons & (1<<1);
    button_E = ControllerData.buttons & (1<<1);
    button_F = ControllerData.buttons & (1<<1);
    button_G = ControllerData.buttons & (1<<1);
    button_H = ControllerData.buttons & (1<<1);

    if(state == 0){
		robot_vx = ((float)(2048 - Leftx))/1000;
    if(robot_vx>0 && robot_vx<1) robot_vx = robot_vx*robot_vx;
    else if(robot_vx>-1 && robot_vx<0) robot_vx = -robot_vx*robot_vx;
	  robot_vy = ((float)(2048 - Lefty))/1000;
    if(robot_vy>0 && robot_vy<1) robot_vy = robot_vy*robot_vy;
    else if(robot_vy>-1 && robot_vy<0) robot_vy = -robot_vy*robot_vy;
    robot_rot = -((float)(Rightx - 2048))/1000;
    if(robot_rot>0 && robot_rot<1) robot_rot = robot_rot*robot_rot;
    else if(robot_rot>-1 && robot_rot<0) robot_rot = -robot_rot*robot_rot;
  //xian fu
		if(robot_rot>2) robot_rot = 2;
    if(robot_rot<-2) robot_rot = -2;
		if(robot_vx>2) robot_vx = 2;
    if(robot_vx<-2) robot_vx = -2;
		if(robot_vy>2) robot_vy = 2;
    if(robot_vy<-2) robot_vy = -2;
  //qu qiu you hua
    if(ADS1256_diff_data[0]<212000)
    {
      robot_vx*=0.3f;
      robot_vy*=0.3f;
    }
    if((ADS1256_diff_data[0]<115000)&&(robot_vy<0)) robot_vy = 0;
    }
    Kine_SetSpeed(robot_vx,robot_vy,robot_rot);
    

		CanTransmit_DJI_1234(&hcan1,
															 hDJI[0].speedPID.output,
															 hDJI[1].speedPID.output,
															 hDJI[2].speedPID.output,
															 hDJI[3].speedPID.output);
		
    if(button_E == 1 && button_B == 1)
    {
      switch (region)
      {
        case 0:
        zone = 25;
        break;
        case 1:
        zone = 1;
        break;
        case 3:
        zone = 3;
        break;
        case 5:
        zone = 5;
        break;
        case 7:
        zone = 7;
        break;
        case 9:
        zone = 9;
        break;
        case 11:
        zone = 11;
        break;
        case 2:
        zone = 13;
        break;
        case 4:
        zone = 15;
        break;
        case 6:
        zone = 17;
        break;
        case 8:
        zone = 19;
        break;
        case 10:
        zone = 21;
        break;
        case 12:
        zone = 23;
        break;
      default:
      zone = zone;
        break;
      }
    }

    if(button_F == 1 && button_B == 1)
    {
      switch (region)
      {
        case 0:
        zone = 0;
        break;
        case 1:
        zone = 2;
        break;
        case 3:
        zone = 4;
        break;
        case 5:
        zone = 6;
        break;
        case 7:
        zone = 8;
        break;
        case 9:
        zone = 10;
        break;
        case 11:
        zone = 12;
        break;
        case 2:
        zone = 14;
        break;
        case 4:
        zone = 16;
        break;
        case 6:
        zone = 18;
        break;
        case 8:
        zone = 20;
        break;
        case 10:
        zone = 22;
        break;
        case 12:
        zone = 24;
        break;
      default:
      zone = zone;
        break;
      }
    }

		nrfDataBag.Leftx = Leftx;
		nrfDataBag.Rightx = Rightx;
		nrfDataBag.Lefty = Lefty;
		nrfDataBag.Righty = Righty;
		nrfDataBag.button_A = button_A;
		nrfDataBag.button_B = button_B;
		nrfDataBag.button_C = button_C;
		nrfDataBag.button_D = button_D;
		nrfDataBag.button_E = button_E;
		nrfDataBag.button_F = button_F;
		nrfDataBag.button_G = button_G;
		nrfDataBag.button_H = button_H;
    nrfDataBag.zone = zone;
		nrfDataBag.qu_qiu = qu_qiu;
		nrfDataBag.she_qiu = she_qiu;
		send();


    if(counter%5 == 0) printf("pgy:%d,%d,%d,%u\n",(int)(hDJI[2].speedPID.ref*100),(int)(hDJI[2].FdbData.rpm *100),(int)(hDJI[2].speedPID.output*100),(HAL_GetTick() - tick));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(1);
    counter++;
		// wheel[0].speed = 	wheel[1].speed = 	wheel[2].speed = 	wheel[3].speed = 0; 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim4))
	{
		time++;
    if ((time % 2) == 1)
    {    
      speedServo(wheel[0].speed,&hDJI[0]);
      speedServo(wheel[1].speed,&hDJI[1]);
      speedServo(wheel[2].speed,&hDJI[2]);
      speedServo(wheel[3].speed,&hDJI[3]);
    }
  }
  if (htim == (&htim5))
	{
		//FSM TRANS BEGIN
		if(button_E == 1 && button_D == 1 && state == 0)
		{
			// enter_time = time;
      // state = 1;
      switch (index_r)
      {
        case 0:
        ref_x = Ref_x[1];
        ref_y = Ref_y[1];
        region = 1;
        break;
        case 1:
        ref_x = Ref_x[3];
        ref_y = Ref_y[3];
        region = 3;
        break; 
        case 2:
        ref_x = Ref_x[5];
        ref_y = Ref_y[5];
        region = 5;
        break;
        case 3:
        ref_x = Ref_x[7];
        ref_y = Ref_y[7];
        region = 7;
        break;
        case 4:
        ref_x = Ref_x[9];
        ref_y = Ref_y[9];
        region = 9;
        break;
        case 5:
        ref_x = Ref_x[11];
        ref_y = Ref_y[11];
        region = 11;
        break;
      
      default:
        // ref_x = ((float)(ADS1256_diff_data[3]))/547098.f;
        // ref_y = ((float)(ADS1256_diff_data[0]))/547098.f;
        ref_x = cur_x;
        ref_y = cur_y;
        region = region;
        break;
      }
    } 
      if(button_F == 1 && button_D == 1 && state == 0)
		{
			// enter_time = time;
      // state = 2;
      switch (index_b)
      {
        case 0:
        ref_x = Ref_x[2];
        ref_y = Ref_y[2];
        region = 2;
        break;
        case 1:
        ref_x = Ref_x[4];
        ref_y = Ref_y[4];
        region = 4;
        break; 
        case 2:
        ref_x = Ref_x[6];
        ref_y = Ref_y[6];
        region = 6;
        break;
        case 3:
        ref_x = Ref_x[8];
        ref_y = Ref_y[8];
        region = 8;
        break;
        case 4:
        ref_x = Ref_x[10];
        ref_y = Ref_y[10];
        region = 10;
        break;
        case 5:
        ref_x = Ref_x[12];
        ref_y = Ref_y[12];
        region = 12;
        break;
      
      default:
        // ref_x = ((float)(ADS1256_diff_data[3]))/547098.f;
        // ref_y = ((float)(ADS1256_diff_data[0]))/547098.f;
        ref_x = cur_x;
        ref_y = cur_y;
        region = region;
        break;
      }
		}

    if(button_A == 1 && state == 0)
	  {
      // enter_time = time;
      // state = 3;
      ref_x = Ref_x[0];
      ref_y = Ref_y[0];
      region = 0;
    }
    if(button_G == 1 && state == 0)
	  {
      enter_time = time;
      state = 4;
    }
    if(button_H == 1 && state == 0)
	  {
      enter_time = time;
      state = 5;
    }
    //FSM TRANS END

    //FSM DO BEGIN
    if((state == 1) || (state == 2) || (state == 3))
    {
      if((cur_x - ref_x) > 1000
      || (cur_x - ref_x) > 1000
      || (cur_y - ref_x) < -1000
      || (cur_y - ref_x) < -1000
      )
      {
        // WTR_MotionPlan_Update(&robot_vx,&robot_vy,time - enter_time,ref_x,ref_y,state);
        /* Bei Yong Fang An*/
        if((cur_x - ref_x) < 1000
        && (cur_x - ref_x) > -1000
        )
        {
          robot_vx = 0;
        }
        if((cur_y - ref_y) < 1000
        && (cur_y - ref_y) > -1000
        )
        {
          robot_vy = 0;
        }
      }
      else
      {
        robot_vx = robot_vy = 0;
        state = 0;
        /* Bei Yong Fang An*/
        // enter_time = time;
        // state = 3;
      }
    }
    /* Bei Yong Fang An*/
    else if(state == 3)
    {
      if (-(enter_time - time)<500)
      {
        robot_vx = robot_vy = 0; 
        /* code */
      }
      else
      {
        state = 0;
      }
    }
    else if(state == 4)
    {
      if((time - enter_time)<175)
      {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
        robot_vx = robot_vy = 0;
      }
//      else if((time - enter_time)<200)
//      {
//        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
//        robot_vx = robot_vy = 0;
//      }
//      else if((time - enter_time)<300)
//      {
//        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
//        robot_vx = robot_vy = 0;
//      }
      else 
      {
        index_r++;
        if(index_r>5) index_r -= 6;
        state = 0;
      }
    }
    else if(state == 5)
    {
      if((time - enter_time)<175)
      {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
        robot_vx = robot_vy = 0;
      }
//      else if((time - enter_time)<200)
//      {
//        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
//        robot_vx = robot_vy = 0;
//      }
//      else if((time - enter_time)<300)
//      {
//        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
//        robot_vx = robot_vy = 0;
//      }
      else 
      {
        index_b++;
        if(index_b>5) index_b -= 6;
        state = 0;
      }
    }
    else
    {
      // 
      
      
      
      
      
      WTR_MotionPlan_Update(&robot_vx,&robot_vy,time - enter_time,ref_x,ref_y,state); 
    }
    //FSM DO END
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart1.Instance)
    {
			  ifRecv = 1;
        WTR_MAVLink_UART_RxCpltCallback(&huart1);
    }
		if(huart->Instance == huart3.Instance)
		{
        ifRecv_mpu = 1;
				mpu6050_decode();
		}
}

/**
 * @brief MAVLink 消息接收完毕回调函数，需要在这里调用解码函数
 *
 * @param msg
 * @return void
 */
void WTR_MAVLink_Msg_RxCpltCallback(mavlink_message_t *msg)
{
	switch (msg->msgid)
	{
	case 1:
		// id = 1 的消息对应的解码函数
    mavlink_msg_controller_decode(msg,&ControllerData);
		break;

	case 2:
		// id = 2 的消息对应的解码函数
		break;
	// ......
	default:
		break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
