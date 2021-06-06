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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BLINK_DEBUG		0
#define PIR_DEBUG		1
#define UART_DEBUG		0
#define TIM_LED_DEBUG	0
#define PWM_DEBUG		0

#define LED_BLINK_PORT		GPIOB
#define LED_BLINK_PIN		GPIO_PIN_7
#define PIR_IN_PORT			GPIOA
#define PIR_IN_PIN			GPIO_PIN_1

#define USART_REC_LEN 15
#define DATA_LEN		4
uint8_t		Res;
uint8_t 	USART1_RX_BUF[USART_REC_LEN];
uint8_t		data_buf[DATA_LEN];
uint8_t 	data_buf_len;
uint8_t  USART1_RX_STA;

enum {
	PRECODE_H = 0,
	PRECODE_L,
	USERCODE_H,
	USERCODE_L,
	CTLCODE_H,
	CTLCODE_L,
	CTLSTOP,
};
struct Remote_Ctl {
	uint8_t state;

	uint16_t usercode;
	uint8_t usercnt;

	uint16_t ctlcode;
	uint8_t ctlcodecnt;
};
#define TURN_OFF_USR_CODE	0xff00
#define TURN_OFF_CTL_CODE	0xba45
static struct Remote_Ctl remote_ctl;

enum {
	NONE_CHK = 0,
	FAST_CHK,
	SLOW_CHK,
};
enum {
	NO_MOVING = 0,
	MOVING,
};
struct Pir_Ctl {
	uint8_t state;
	uint8_t chk_result;
	uint16_t cycle_cnt;
};
#define FAST_CHK_TIME	3//means check every 1 second
#define SLOW_CHK_TIME	1//means check every 1 minute
static struct Pir_Ctl pir_ctl;

uint32_t Tim_Arr_backup = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  Tim_Arr_backup = TIM3->ARR;
  HAL_UART_Receive_IT(&huart2, &Res, 1);

  remote_ctl.state = PRECODE_H;
  remote_ctl.usercode = TURN_OFF_USR_CODE;
  remote_ctl.ctlcode = TURN_OFF_CTL_CODE;

#if (TIM_LED_DEBUG || PWM_DEBUG)
  HAL_TIM_Base_Start_IT(&htim3);
#endif

  pir_ctl.state		= NONE_CHK;
  pir_ctl.chk_result = NO_MOVING;
  pir_ctl.cycle_cnt		= 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if PIR_DEBUG
	  HAL_Delay(20);
	  if(HAL_GPIO_ReadPin(PIR_IN_PORT, PIR_IN_PIN))
		  HAL_GPIO_WritePin(LED_BLINK_PORT, LED_BLINK_PIN, GPIO_PIN_RESET);
	  else
		  HAL_GPIO_WritePin(LED_BLINK_PORT, LED_BLINK_PIN, GPIO_PIN_SET);
#endif
#if BLINK_DEBUG
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(LED_BLINK_PORT, LED_BLINK_PIN);
#endif
#if UART_DEBUG
	  if(USART1_RX_STA & 0x80)
	  {
		  HAL_UART_Transmit_IT(&huart2,USART1_RX_BUF,USART1_RX_STA&0x7F);
		  while(huart2.gState != HAL_UART_STATE_READY){};
		  HAL_UART_Transmit_IT(&huart2,data_buf,data_buf_len);
		  while(huart2.gState != HAL_UART_STATE_READY){};
		  if(data_buf[0] == 0x01)
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		  else if(data_buf[0] == 0x00)
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		  for(i = 0;i < USART_REC_LEN; ++i)
			  USART1_RX_BUF[i] = 0;
		  USART1_RX_STA=0;
	  }
	  HAL_Delay(100);
#endif
	  if(USART1_RX_STA & 0x80)
	  {
		  if((data_buf[0] & 0x01) == 0x00) {//turn off the AC
			  //if(HAL_GPIO_ReadPin(PIR_IN_PORT, PIR_IN_PIN))
				  //pir_ctl.state = FAST_CHK;
			if(!HAL_GPIO_ReadPin(PIR_IN_PORT, PIR_IN_PIN))
			  HAL_TIM_Base_Start_IT(&htim3);
		  } else if((data_buf[0] & 0x01) == 0x01) {//other action
			  remote_ctl.state = PRECODE_H;
			  remote_ctl.usercode = TURN_OFF_USR_CODE;
			  remote_ctl.ctlcode = TURN_OFF_CTL_CODE;

			  //HAL_TIM_Base_Start_IT(&htim3);
		  }
		  for(i = 0;i < USART_REC_LEN; ++i)
			  USART1_RX_BUF[i] = 0;
		  USART1_RX_STA=0;
	  }
	  HAL_Delay(10);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if(huart ->Instance == USART2) {
		if((USART1_RX_STA&0x80)==0)
		{
			USART1_RX_BUF[USART1_RX_STA&0X3F] = Res ;
			USART1_RX_STA++;
			if(USART1_RX_STA > (USART_REC_LEN-1))
				USART1_RX_STA=0;

			if(Res==0xFF) {
				data_buf_len = USART1_RX_BUF[1] - 4;
				for(i = 0;i < data_buf_len; ++i)
					data_buf[i] = USART1_RX_BUF[6+i];
				USART1_RX_STA|=0x80;
			}

			HAL_UART_Receive_IT(huart,&Res,1);
		}
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim3)) {
#if TIM_LED_DEBUG
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		TIM3->ARR = 50000 - 1;
#endif
		if(pir_ctl.state == NONE_CHK) {
			switch(remote_ctl.state){
			case PRECODE_H:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				TIM3->ARR = 900 - 1;//9ms
				remote_ctl.state = PRECODE_L;
				break;
			case PRECODE_L:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				TIM3->ARR = 450 - 1;//4.5ms
				remote_ctl.state = USERCODE_H;
				break;
			case USERCODE_H:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				TIM3->ARR = 56 - 1;//0.56ms
				remote_ctl.state = USERCODE_L;
				break;
			case USERCODE_L:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				if(remote_ctl.usercode >> remote_ctl.usercnt & 0x01)
					TIM3->ARR = 169 - 1;//1.69ms
				else
					TIM3->ARR = 56 - 1;//0.565ms
				remote_ctl.usercnt++;
				if(remote_ctl.usercnt >= 16){
					remote_ctl.usercnt = 0;
					remote_ctl.state = CTLCODE_H;
				} else {
					remote_ctl.state = USERCODE_H;
				}
				break;
			case CTLCODE_H:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				TIM3->ARR = 56 - 1;//0.56ms
				remote_ctl.state = CTLCODE_L;
				break;
			case CTLCODE_L:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				if(remote_ctl.ctlcode >> remote_ctl.ctlcodecnt & 0x01)
					TIM3->ARR = 169 - 1;//1.69ms
				else
					TIM3->ARR = 56 - 1;//0.565ms
				remote_ctl.ctlcodecnt++;
				if(remote_ctl.ctlcodecnt >= 17){
					remote_ctl.ctlcodecnt = 0;
					remote_ctl.state = CTLSTOP;
				} else {
					remote_ctl.state = CTLCODE_H;
				}
				break;
			case CTLSTOP:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				remote_ctl.state = PRECODE_H;
				HAL_TIM_Base_Stop_IT(&htim3);
				break;
			default:
				break;
			}
		} else if(pir_ctl.state == FAST_CHK) {
/*			if(pir_ctl.cycle_cnt >= 1)
				pir_ctl.chk_result += HAL_GPIO_ReadPin(PIR_IN_PORT, PIR_IN_PIN);

			if(pir_ctl.cycle_cnt == FAST_CHK_TIME && pir_ctl.chk_result == NO_MOVING) {//NO MOVING
				pir_ctl.state = NONE_CHK;
				pir_ctl.cycle_cnt = 0;
				pir_ctl.chk_result == NO_MOVING;

				remote_ctl.state = PRECODE_H;
				remote_ctl.usercode = TURN_OFF_USR_CODE;
				remote_ctl.ctlcode = TURN_OFF_CTL_CODE;

				TIM3->ARR = Tim_Arr_backup;
				return;
			} else {//MOVING
				pir_ctl.cycle_cnt = 0;
			}
			TIM3->ARR = 50000;
			pir_ctl.cycle_cnt++;*/
		} else if(pir_ctl.state == SLOW_CHK) {
			//TIM3->ARR = 50000;
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
