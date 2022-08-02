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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t flag_sim_IT=0;
uint8_t buffer[32];
uint8_t rev3_IT;
uint8_t _index=0;

uint8_t cmd_sim808_check[10] = "AT"; // check connect MCU with module sim808
uint8_t cmd_sim808_echo[15] = "ATE0"; // turn off echo mode
uint8_t cmd_sim808_sset[15] = "AT&W"; // save setting
uint8_t cmd_sim808_SQualityCheck[15] = "AT+CSQ";
uint8_t cmd_sim808_SimInfor[15] = "AT+CCID";
uint8_t cmd_sim808_GetModuleName[4] = "ATI";
uint8_t cmd_sim808_CrcNetwork[15] = "AT+7COSP?";
uint8_t cmd_sim808_CheckSIM[15] = "AT+CPIN?";


uint8_t cmd_sim808_inforcall[15] = "AT+CLIP=1"; // show information call
uint8_t cmd_sim808_call[20] = "ATD0369502269;"; // call from number phone
uint8_t cmd_sim808_ATH[15] = "ATH"; // end call

uint8_t cmd_sim808_offgps[15] = "AT+CGNSPWR=0"; // turn off gps mode
uint8_t cmd_sim808_ongps[15] = "AT+CGNSPWR=1"; // turn on gps mode
uint8_t cmd_sim808_getgps[20] = "ART+CGNSINF=32"; // show location
uint8_t cmd_sim808_gps[15] = "AT+CGNSURC=1"; // set time =1s show location 

uint8_t cmd_sim808_bt_of[15] = "AT+BTPOWE7R=0"; // turn off bluetooth mode
uint8_t cmd_sim808_bt_on[15] = "AT+BTPOWER=1"; // turn on bluetooth mode
uint8_t cmd_sim808_btpair[15] = "AT+BTPAIR=1,1"; // agree connecting with smartphone
uint8_t cmd_sim808_btacpt[15] = "AT+BTACPT=1"; // charge profine mode
uint8_t cmd_sim808_btgetprof[15] = "AT+BTGETPROF=1"; // check profine mode
uint8_t cmd_sim808_btconn[15] = "AT+BTCONN=1,4"; // connect profine SSP mode
//AT+BTSPPSEND=16 transmit data to smartphone

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void send_sim808(uint8_t *string);
static void send_modbus(uint8_t *string);
static void waitRespone(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char apn[]  = "v-internet"; // Change this to your Provider details
const char server[] = "dhikon.atwebpages.com/"; // Change this to your domain
const int  port = 80;
const char resource[] = "/insert.php";
const uint32_t timeOut =10000;
char content[80];
char ATcommand[80];
uint8_t _buffer[100] = {0};
uint8_t ATisOK = 0;
uint8_t CGREGisOK = 0;
uint8_t CIPOPENisOK = 0;
uint8_t NETOPENisOK = 0;
uint32_t previousTick;
uint16_t distance;

void SIMTransmit(char *cmd)
{
  memset(buffer,0,sizeof(buffer));
  HAL_UART_Transmit(&huart1,(uint8_t *)cmd,strlen(cmd),1000);
  HAL_UART_Receive (&huart1, buffer, 100, 1000);
}

void httpPost(void)
{
  ATisOK = 0;
  CGREGisOK = 0;
  NETOPENisOK = 0;
  CIPOPENisOK = 0;
  // Check for OK response for AT 
  previousTick =  HAL_GetTick();
  while(!ATisOK && previousTick  + timeOut >  HAL_GetTick())
  {
    SIMTransmit("AT\r\n");
    HAL_Delay(1000);
    if(strstr((char *)buffer,"OK"))
    {
      ATisOK = 1;
    }
  }

  // Check for network registration. 
  if(ATisOK)
  {
    previousTick =  HAL_GetTick();
    while(!CGREGisOK  && previousTick  + timeOut >  HAL_GetTick())
    {
      SIMTransmit("AT+CGREG?\r\n");
      if(strstr((char *)buffer,"+CGREG: 0,1"))
      {
        CGREGisOK = 1;
      }
    }
  }

  // Check for Internet IP Connection
  if(ATisOK && CGREGisOK)
  {
    previousTick =  HAL_GetTick();
    while(!NETOPENisOK  && previousTick  + timeOut >  HAL_GetTick())
    {
      SIMTransmit("AT+NETCLOSE\r\n");
      sprintf(ATcommand,"AT+CGDCONT=1,\"IP\",\"%s\",\"0.0.0.0\",0,0\r\n",apn);
      SIMTransmit(ATcommand);
      SIMTransmit("AT+CIPMODE=0\r\n");
      SIMTransmit("AT+CIPSENDMODE=0\r\n");
      SIMTransmit("AT+CIPCCFG=10,0,0,0,1,0,75000\r\n");
      SIMTransmit("AT+CIPTIMEOUT=75000,15000,15000\r\n");
      SIMTransmit("AT+NETOPEN\r\n");
      SIMTransmit("AT+NETOPEN?\r\n");
      if(strstr((char *)buffer,"+NETOPEN: 1"))
      {
        NETOPENisOK = 1;
      }
    }
  }

  // Check for TCP connection
  if(ATisOK && CGREGisOK && NETOPENisOK)
  {
    SIMTransmit("AT+IPADDR\r\n");
    previousTick =  HAL_GetTick();
    while(!CIPOPENisOK  && previousTick  + timeOut >  HAL_GetTick())
    {
      SIMTransmit("AT+CIPCLOSE=0\r\n");
      sprintf(ATcommand,"AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n",server,port);
      SIMTransmit(ATcommand);
      HAL_Delay(1000);
      if(strstr((char *)buffer,"+CIPOPEN: 0,0"))
      {
        CIPOPENisOK = 1;
      }
    }
  }

  // If all Connection success (Wiring, Registration and TCP/IP)
  if(ATisOK && CGREGisOK && CIPOPENisOK && NETOPENisOK)
  {
    // Perform http request
    sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(resource)+16);
    SIMTransmit(ATcommand);
    if(strstr((char *)buffer,">"))
    {
      sprintf(ATcommand,"POST %s HTTP/1.1\r\n",resource);
      SIMTransmit(ATcommand);
    }

    sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(server)+8);
    SIMTransmit(ATcommand);
    if(strstr((char *)buffer,">"))
    {
      sprintf(ATcommand,"Host: %s\r\n",server);
      SIMTransmit(ATcommand);
    }

    SIMTransmit("AT+CIPSEND=0,19\r\n");
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit("Connection: close\r\n");
    }

    SIMTransmit("AT+CIPSEND=0,49\r\n");
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit("Content-Type: application/x-www-form-urlencoded\r\n");
    }

    SIMTransmit("AT+CIPSEND=0,16\r\n");
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit("Content-Length: ");
    }

    char sLength[5];
    snprintf(sLength, 5,"%d",strlen(content));
    sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(sLength));
    SIMTransmit(ATcommand);
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit(sLength);
    }

    SIMTransmit("AT+CIPSEND=0,2\r\n");
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit("\r\n");
    }

    SIMTransmit("AT+CIPSEND=0,2\r\n");
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit("\r\n");
    }

    sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(content));
    SIMTransmit(ATcommand);
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit(content);
    }

    SIMTransmit("AT+CIPSEND=0,2\r\n");
    if(strstr((char *)buffer,">"))
    {
      SIMTransmit("\r\n");
    }
    HAL_Delay(2000);
    // Close connections
    SIMTransmit("AT+CIPCLOSE=0\r\n");
    SIMTransmit("AT++NETCLOSE\r\n");
  }
}
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3,&rev3_IT,1);
  /*****************************/
  /* RESET MODULE SIM 808 */
//  HAL_GPIO_WritePin(PRST_GPIO_Port,PRST_Pin,1);
//  HAL_Delay(2000);
//  HAL_GPIO_WritePin(PRST_GPIO_Port,PRST_Pin,0);
//  HAL_Delay(2000);
  /*****************************/
  send_modbus(cmd_sim808_check);
  HAL_Delay(1000);
  /*****************************/
  /* CHECK CMD*/
  send_sim808(cmd_sim808_check);

  /*****************************/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  waitRespone();
	  distance ++;
	  sprintf(content,"key=a@4K3&distance=%d",distance);
      httpPost();
	  HAL_GPIO_TogglePin(Led_Status_GPIO_Port,Led_Status_Pin);
	  HAL_Delay(2000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  huart2.Init.BaudRate = 115200;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PRST_GPIO_Port, PRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Led_Status_Pin|Led_Power_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PRST_Pin */
  GPIO_InitStruct.Pin = PRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_Status_Pin Led_Power_Pin */
  GPIO_InitStruct.Pin = Led_Status_Pin|Led_Power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void send_sim808(uint8_t *string)
{
	char buff[32];
	sprintf((char*)&buff[0],"%s\r\n",string);
	uint8_t len = strlen((char*)&buff[0]);
	HAL_UART_Transmit(&huart3,(uint8_t*)&buff[0],len,100);
	for(int i = 0; i <= 32; i++)
	{buff[i] = NULL;}
}
static void send_modbus(uint8_t *string)
{
	char buff1[32];
	sprintf((char*)&buff1[0],"%s\r\n",string);
	uint8_t len = strlen((char*)&buff1[0]);
	HAL_UART_Transmit(&huart1,(uint8_t*)&buff1[0],len,100);
	for(int i = 0; i <= 32; i++)
	{buff1[i] = NULL;}
}
static void waitRespone(void)
{
	switch(flag_sim_IT)
	{
		case  0:
			break;
		case 1:
			HAL_UART_Receive(&huart1,&rev3_IT,sizeof(rev3_IT),100);
			for(int i =0;i<=32;i++)
			{
				buffer[i] = NULL;
			}
			flag_sim_IT = 0;
			break;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
