/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum led_mode {
	MODE_AUTO,
	MODE_MANUAL
} LED_MODE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t start_sign_received = 0;
volatile uint8_t command_buffer[10];
volatile uint8_t command_buffer_pos = 0;
volatile LED_MODE mode = MODE_AUTO;
volatile uint8_t led_target_duty = 0;
volatile uint8_t led_real_duty = 0;
volatile uint8_t direction = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setDutyCycle(uint8_t D);
void proccesDmaData(uint8_t sign);
void update_PWM();
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  USART2_RegisterCallback(proccesDmaData);

  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM2);

  TIM2CC2_register_callback(update_PWM);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableIT_CC2(TIM2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 4 */
void proccesDmaData(uint8_t sign)
{
	if((sign == '\n') || (sign == '\r') || (sign == '\0'))
	{
		return;
	}

	if(!start_sign_received)
	{
		if(sign == '$')
		{
			start_sign_received = 1;
			command_buffer_pos = 0;
			return;
		}
	}
	else
	{
		if(sign == '$')
		{
			start_sign_received = 0;

			if(!strncmp(command_buffer, "manual", 6))
			{
				mode = MODE_MANUAL;
				return;
			}

			if(!strncmp(command_buffer, "auto", 4))
			{
				mode = MODE_AUTO;
				return;
			}

			if(!strncmp(command_buffer, "PWM", 3))
			{
				int8_t tens = command_buffer[3] - '0';
				int8_t ones = command_buffer[4] - '0';

				if((ones > 9) || (tens > 9) || (ones < 0) || (tens < 0))
				{
					return;
				}

				led_target_duty = tens*10 + ones;
			}
			return;
		}

		command_buffer[command_buffer_pos] = sign;
		command_buffer_pos++;
	}
}

void setDutyCycle(uint8_t D)
{
	uint8_t pulse_length;
	pulse_length = ((TIM2->ARR) * D) / 100;
	TIM2->CCR1 = pulse_length;
}

void update_PWM()
{
	if(mode==MODE_MANUAL)
	{
		if(led_real_duty<led_target_duty)
		{
			led_real_duty++;
		}

		if(led_real_duty>led_target_duty)
		{
			led_real_duty--;
		}
	}

	if(mode==MODE_AUTO)
	{
		if(direction==0)
		{
			led_real_duty++;
			if(led_real_duty==99)
			{
				direction=1;
			}
		}

		else if(direction==1)
		{
			led_real_duty--;
			if(led_real_duty==0)
			{
				direction=0;
			}
		}
	}

	setDutyCycle(led_real_duty);
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
