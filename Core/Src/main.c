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

#include "stdio.h"  
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int ifRecv;

uint32_t time = 0;
uint32_t enter_time = 0;

int b = 0;
int a = 0;
int state = 0;

uint8_t zone = 0;
uint8_t qu_qiu = 0;
uint8_t she_qiu = 0;

float robot_vx = 0;
float robot_vy = 0;
float robot_rot = 0;

int32_t Bias_mpu = 0;
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
  // MotionPlan_Init()
	
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	
	nrf_Transmit_init();
	nrf_receive_init();

	mpu6050_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		while(!ifRecv)
		{
		}
		ADS1256_UpdateDiffData();
		
    if(state == 0){
		robot_vx = ((float)(2048 - Leftx))/1000;
	  robot_vy = ((float)(2048 - Lefty))/1000;
	  robot_rot = -((float)(Rightx - 2048))/1000;
    }
    else{
      if(Bias_mpu>5 || Bias_mpu<5)robot_rot = 3*Bias_mpu;
    }

		Kine_SetSpeed(robot_vx,robot_vy,robot_rot);
		
		speedServo(wheel[0].speed,&hDJI[0]);
		speedServo(wheel[1].speed,&hDJI[1]);
		speedServo(wheel[2].speed,&hDJI[2]);
		speedServo(wheel[3].speed,&hDJI[3]);
		
		CanTransmit_DJI_1234(&hcan1,
															 hDJI[0].speedPID.output,
															 hDJI[1].speedPID.output,
															 hDJI[2].speedPID.output,
															 hDJI[3].speedPID.output);
		
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
		
		Bias_mpu += mpu6050_databag.Wz;

    // printf("%d\r\n",time);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(1);
		wheel[0].speed = 	wheel[1].speed = 	wheel[2].speed = 	wheel[3].speed = 0; 
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
	}
  if (htim == (&htim5))
	{
		//FSM TRANS BEGIN
		if(button_D == 1 && state == 0)
		{
			enter_time = time;
      state = 1;
		}
    //FSM TRANS END

    //FSM DO BEGIN
    if(state == 1)
    {
      if((time - enter_time)<1000)
      {
        robot_vy = ((float)(time-enter_time))/1000;
      }
      else if((time - enter_time)<2000)
      {
        robot_vy = -((float)(time-enter_time))/1000 + 2;
      }
      else
      {
        state = 0;
      }
    }
    //FSM DO END
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart6.Instance)
    {
			  ifRecv = 1;
        nrf_decode();
    }
		if(huart->Instance == huart3.Instance)
		{
				mpu6050_decode(&mpu6050_databag);
		}
}

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart6,(uint8_t *)&ch,1,0xFFFF);//阻塞方式打印
  return ch;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
