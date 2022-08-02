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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Begin interrupt UART1 */
uint8_t rev_data;
uint8_t buffer_modbus[8];
//uint8_t modbus_index = 0;
uint8_t flag_modbus = 0;
//uint8_t flag_crc = 0;
uint8_t Pow_ms;
uint8_t Stt_ms;
/* End interrupt UART1 */
/* Begin frame Modbus RTU */
#define ID_S1000 0x01
#define Function_RO 0x01
#define Function_WO 0x02
#define Function_RW 0x03
#define NoUse 0x00
#define Add_Volt_Solar 0x0001
#define Add_Volt_SAcquy 0x0002
#define Add_Volt_Wacquy 0x0003
#define Add_Ampe_Solar 0x0004
#define Add_Ampe_Sacquy 0x0005
#define Add_Ampe_Wacquy 0x0006
#define Add_Wat_Solar 0x0007
#define Add_Wat_Sacquy 0x0008
#define Add_Wat_Wacquy 0x0009
#define Add_Wat_S1000 0x00A1
#define Add_Wat_W1000 0x00A2
#define Add_Lever_Sensor 0x00A3
#define Add_Temp_Sensor 0x00A4
#define Add_Press_Senser 0x00A5
#define Add_Control_Load 0x0001
#define Add_Charging_Mode 0x0002
#define Add_Brake_Turbin 0x0003
#define Data_Reg_Fixed 0x0001
#define Data_Reg_OFF 0x0001
#define Date_Reg_ON 0x0002

typedef struct{
	uint8_t Add_ID;
	uint8_t Function_code;
	uint8_t MSB_AddReg;
	uint8_t LSB_AddReg;
	uint8_t MSB_DataReg;
	uint8_t LSB_DataReg;
	uint16_t AddRreg;
	uint16_t DataReg;
}ModbusRTU_STRUCT;
ModbusRTU_STRUCT ModbusRTU;

typedef struct{
	uint8_t data_Volt_Solar;
	uint8_t data_Volt_SAcquy;
	uint8_t data_Volt_Wacquy;
	uint8_t data_Ampe_Solar;
	uint8_t data_Ampe_Sacquy;
	uint8_t data_Ampe_Wacquy;
	uint8_t data_Wat_Solar;
	uint8_t data_Wat_Sacquy;
	uint8_t data_Wat_Wacquy;
	uint8_t data_Wat_S1000;
	uint8_t data_Wat_W1000;
	uint8_t data_Lever_Sensor;
	uint8_t data_Temp_Sensor;
	uint8_t data_Press_Senser;
}ADC_STRUCT;
ADC_STRUCT value_adc;

/* End frame Modbus RTU */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void check_crc(void);
void Write_Mobus_RTU(uint8_t byte_ID, uint8_t byte_func, uint16_t byte_addReg, uint16_t byte_dataReg);
static void Clear_Modbus(void);
static void Read_ADC(void);

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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_IT(&huart1,&rev_data,1);
  HAL_GPIO_WritePin(Led_Pow_GPIO_Port,Led_Pow_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Led_Status_GPIO_Port,Led_Status_Pin,GPIO_PIN_SET);
  HAL_Delay(1000);
//  Write_Mobus_RTU(ID_S1000,Function_RO,Add_Ampe_Sacquy,Data_Reg_Fixed);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Read_ADC();
	  check_crc();
	  switch(ModbusRTU.Function_code)
	  {
		  case 0x01: // read only
			  if(ModbusRTU.Add_ID == 0xAA)
			  {
				  HAL_Delay(1);
				  uint16_t add_id = ID_S1000 << 8;
				  Write_Mobus_RTU(ModbusRTU.Add_ID,Function_RO,add_id,0x0000); // S-1000
 
				  Clear_Modbus();
			  }
			  break;
		  case 0x02: // write only
			  break;
		  case 0x03: // read write
			  break;
		  default:
			  break;
	  }
	  
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Led_Pow_Pin|Led_Status_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pow_Pin */
  GPIO_InitStruct.Pin = Led_Pow_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_Pow_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_Status_Pin */
  GPIO_InitStruct.Pin = Led_Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_Status_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void Clear_Modbus(void)
{
	ModbusRTU.Add_ID = NULL;
	ModbusRTU.Function_code = NULL;
	ModbusRTU.MSB_AddReg = NULL;
	ModbusRTU.LSB_AddReg = NULL;
	ModbusRTU.MSB_DataReg = NULL;
	ModbusRTU.LSB_DataReg = NULL;
	ModbusRTU.AddRreg = NULL;
	ModbusRTU.DataReg =NULL;
}
static void check_crc(void)
{
	uint8_t _crc[1];
	uint8_t _code_error[1] = {0xA2};
	switch(flag_modbus)
	{
		case 0:
			// nothing
			break;
		case 1:
			// check sum
//			HAL_UART_Transmit(&huart1,buffer_modbus,sizeof(buffer_modbus),100);
			_crc[0] = buffer_modbus[0] + buffer_modbus[1] + buffer_modbus[2] + buffer_modbus[3] + buffer_modbus[4] + buffer_modbus[5];
//		    HAL_UART_Transmit(&huart1,check_crc,sizeof(check_crc),100);
			if(_crc[0] == buffer_modbus[6])
			{
				ModbusRTU.Add_ID = buffer_modbus[0];
				ModbusRTU.Function_code = buffer_modbus[1];
				ModbusRTU.MSB_AddReg = buffer_modbus[2];
				ModbusRTU.LSB_AddReg = buffer_modbus[3];
				ModbusRTU.MSB_DataReg = buffer_modbus[4];
				ModbusRTU.LSB_DataReg = buffer_modbus[5];
				ModbusRTU.AddRreg = ModbusRTU.MSB_AddReg << 8 | ModbusRTU.LSB_AddReg;
				ModbusRTU.DataReg = ModbusRTU.MSB_DataReg << 8 | ModbusRTU.LSB_DataReg;
				_crc[0] =NULL;
			}
//			strcpy
			else /* if(check_crc[0] != buffer_modbus[6])*/
			{
				HAL_UART_Transmit(&huart1,_code_error,sizeof(_code_error),100);
//				status_blynk();
//				Stt_ms = 1;
			}
			flag_modbus = 0;
			Stt_ms = 1;
			for(int i=0;i<=8;i++)
				buffer_modbus[i]=NULL;
			break;
		default:
			break;
	}
}
void Write_Mobus_RTU(uint8_t byte_ID, uint8_t byte_func, uint16_t byte_addReg, uint16_t byte_dataReg)
{
	uint8_t Add_MSB = byte_addReg >> 8;
	uint8_t Add_LSB = byte_addReg;
	uint8_t data_MSB = byte_dataReg >> 8;
	uint8_t data_LSB = byte_dataReg;
	uint8_t byte_crc = byte_ID+byte_func+Add_MSB+Add_LSB+data_MSB+data_LSB;
	uint8_t byte_start = 0x3A;
	uint8_t byte_stop[2] = {0x0D,0x0A};
	uint8_t Write_Modbus[11] = {byte_start,byte_ID,byte_func,Add_MSB,Add_LSB,data_MSB,data_LSB,byte_crc,byte_stop[0],byte_stop[1]};
	HAL_UART_Transmit(&huart1,Write_Modbus,sizeof(Write_Modbus),1000);
	for(int i=0;i<=11;i++)Write_Modbus[i]=NULL;
}

static void Read_ADC(void)
{
	value_adc.data_Ampe_Sacquy = 25; // I = 25A
	value_adc.data_Ampe_Solar = 26;
	value_adc.data_Ampe_Wacquy = 27;
	value_adc.data_Lever_Sensor = 1;
	value_adc.data_Press_Senser = 1;
	value_adc.data_Temp_Sensor = 1;
	value_adc.data_Volt_SAcquy = 27;
	value_adc.data_Volt_Solar = 35;
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
