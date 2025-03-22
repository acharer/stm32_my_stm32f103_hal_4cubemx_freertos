/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include <string.h>
#include "ap3216c.h"
#include "socket.h"
#include "DHCP/dhcp.h"
#include "DNS/dns.h"
#include "bsp_spi_flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/* ---------------------------------------- task handles -------------------------------------------------- */
static osThreadId len_run_task_Handle; /* len_run_task handle */ 
static osThreadId key_scan_task_Handle; /* key_scan_task handle */ 
static osThreadId get_i2c_data_task_Handle; /* get_i2c_data_task handle */ 
static osThreadId get_adc_data_task_Handle; /* get_adc_data_task handle */ 
static osThreadId print_data_task_Handle; /* print_data_task handle */ 
static osThreadId network_task_Handle; /* network_task handle */ 
static osThreadId spi_flash_test_task_Handle; /* spi_flash_test_task handle */ 


/* ---------------------------------------- data variables -------------------------------------------------- */
static uint32_t adcValue;
float user_ALS;
uint16_t user_PS;
uint16_t user_IR; 
uint64_t g_time_ms;

/* ---------------------------------------- network variables ---------- */
uint8_t dhcp_buffer[1024]; // 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
uint8_t dns_buffer[1024]; // 1K seems to be enough for this buffer as well
volatile bool ip_assigned = false;

void CheckHeapSize(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void led_run_task(void const * argument);/* len_run_task function */
static void key_scan_task(void const * argument);/* key_scan_task function */
static void get_i2c_data_task(void const * argument);/* get_i2c_data_task function */
static void get_adc_data_task(void const * argument);/* get_adc_data_task function */
static void print_data_task(void const * argument);/* print_data_task function */
static void network_task(void const * argument);/* network_task function */
static void spi_flash_test_task(void const * argument);/* spi_flash_test_task function */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  ap3216c_init();
  SPI_FLASH_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(LedRunTask, led_run_task, osPriorityNormal, 1, 128);
  len_run_task_Handle = osThreadCreate(osThread(LedRunTask), NULL);

  osThreadDef(KeyScanTask, key_scan_task, osPriorityNormal, 2, 128);
  key_scan_task_Handle = osThreadCreate(osThread(KeyScanTask), NULL);

  osThreadDef(GetI2cDataTask, get_i2c_data_task, osPriorityNormal, 1, 128);
  get_i2c_data_task_Handle = osThreadCreate(osThread(GetI2cDataTask), NULL);

  osThreadDef(GetAdcDataTask, get_adc_data_task, osPriorityNormal, 1, 128);
  get_adc_data_task_Handle = osThreadCreate(osThread(GetAdcDataTask), NULL);

  osThreadDef(PrintDataTask, print_data_task, osPriorityNormal, 1, 128);
  print_data_task_Handle = osThreadCreate(osThread(PrintDataTask), NULL);

//  osThreadDef(NetworkTask, network_task, osPriorityNormal, 1, 1024);
//  network_task_Handle = osThreadCreate(osThread(NetworkTask), NULL);

//  osThreadDef(SpiFlashTestTask, spi_flash_test_task, osPriorityNormal, 1, 128);
//  spi_flash_test_task_Handle = osThreadCreate(osThread(SpiFlashTestTask), NULL);
 



  CheckHeapSize();
  printf("Start scheduler\r\n");
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    osThreadTerminate(len_run_task_Handle);
    osThreadTerminate(key_scan_task_Handle);
    osThreadTerminate(get_i2c_data_task_Handle);
    osThreadTerminate(get_adc_data_task_Handle);
    osThreadTerminate(print_data_task_Handle);
    osThreadTerminate(network_task_Handle);
    osThreadTerminate(spi_flash_test_task_Handle);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AP_EN_GPIO_Port, AP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BEEP_Pin|RS485_EN_SEND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_FALSH_CS_GPIO_Port, SPI_FALSH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY2_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AP_EN_Pin */
  GPIO_InitStruct.Pin = AP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AP_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BEEP_Pin RS485_EN_SEND_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin|RS485_EN_SEND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_FALSH_CS_Pin */
  GPIO_InitStruct.Pin = SPI_FALSH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_FALSH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_BLUE_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DETECT_SDIO_Pin */
  GPIO_InitStruct.Pin = DETECT_SDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DETECT_SDIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*redefine fputc function to transmit data through uart1*/
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);     

    USART1->DR = (uint8_t)ch;             
    return ch;
}
/* -----------------------------------------------user task--------------------------------------------------*/
void led_run_task(void const * argument)
{
  for(;;)
  {
    LED_RED_TOGGLE;
 
    osDelay(500);
  } 
}

void key_scan_task(void const * argument)
{
  for(;;)
  {

    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
    {
      osDelay(5);
      if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
      {
        BEEP_ON;
        osDelay(50);
        LED_GREEN_TOGGLE;
        BEEP_OFF;
      }
    }

    if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET)
    {
      osDelay(5);
      if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)  
      {
        BEEP_ON;
        osDelay(50);
        LED_BLUE_TOGGLE;
        BEEP_OFF;
      }
    }

    osDelay(1);
  } 
}

void get_i2c_data_task(void const * argument)
{
  for(;;)
  {
    user_ALS = ap3216c_read_ambient_light();
    user_PS = ap3216c_read_ps_data();
    user_IR = ap3216c_read_ir_data();

    osDelay(500);
  } 
}

void get_adc_data_task(void const * argument)
{
  for(;;)
  {
    HAL_ADC_Start_IT(&hadc1); 

    osDelay(500);
  } 
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adcValue = HAL_ADC_GetValue(hadc); 
    }
}

void print_data_task(void const * argument)
{
  for(;;)
  {
    //printf("ADC Value: %d\r\n", adcValue);
    //printf("light:%.2fLux, red_light:%d, close:%d\r\n", user_ALS, user_IR, user_PS & 0x3FF);

    osDelay(500);
  } 
}

void CheckHeapSize(void)
{
    size_t freeHeapSize = xPortGetFreeHeapSize();
    size_t minFreeHeapSize = xPortGetMinimumEverFreeHeapSize();

    printf("Current free heap size: %zu bytes\n", freeHeapSize);
    printf("Minimum ever free heap size: %zu bytes\n", minFreeHeapSize);
}

void W5500_Select(void) {
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
  HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
  HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void) {
  uint8_t byte;
  W5500_ReadBuff(&byte, sizeof(byte));
  return byte;
}

void W5500_WriteByte(uint8_t byte) {
  W5500_WriteBuff(&byte, sizeof(byte));
}

void Callback_IPAssigned(void) {
  printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
    ip_assigned = true;
}
 
void Callback_IPConflict(void) {
  printf("Callback: IP conflict!\r\n");
}

void network_task(void const * argument)
{
  osDelay(2000);
  printf("\r\ninit() called!\r\n");
  printf("Registering W5500 callbacks...\r\n");
  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
  reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
  reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

  printf("Calling wizchip_init()...\r\n");
  uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
  wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

  printf("Calling DHCP_init()...\r\n");
  wiz_NetInfo net_info = {
      .mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
      .dhcp = NETINFO_DHCP
  };

  // set MAC address before using DHCP
  setSHAR(net_info.mac);
  DHCP_init(DHCP_SOCKET, dhcp_buffer);

  printf("Registering DHCP callbacks...\r\n");
  reg_dhcp_cbfunc(
      Callback_IPAssigned,
      Callback_IPAssigned,
      Callback_IPConflict
  );

  printf("Calling DHCP_run()...\r\n");
  // actually should be called in a loop, e.g. by timer
  uint32_t ctr = 10000;
  while((!ip_assigned) && (ctr > 0)) {
      DHCP_run();
      osDelay(1);
      ctr--;
  }
  if(!ip_assigned) {
    printf("\r\nIP was not assigned :(\r\n");
    //return;
  }

  getIPfromDHCP(net_info.ip);
  getGWfromDHCP(net_info.gw);
  getSNfromDHCP(net_info.sn);

  uint8_t dns[4];
  getDNSfromDHCP(dns);

  printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nDNS: %d.%d.%d.%d\r\n",
      net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
      net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
      net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3],
      dns[0], dns[1], dns[2], dns[3]
  );

  printf("Calling wizchip_setnetinfo()...\r\n");
  wizchip_setnetinfo(&net_info);

  printf("Calling DNS_init()...\r\n");
  DNS_init(DNS_SOCKET, dns_buffer);

  uint8_t addr[4];
  {
      char domain_name[] = "eax.me";
      printf("Resolving domain name \"%s\"...\r\n", domain_name);
      int8_t res = DNS_run(dns, (uint8_t*)&domain_name, addr);
      if(res != 1) {
        printf("DNS_run() failed, res = %d", res);
          return;
      }
      printf("Result: %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
  }

  printf("Creating socket...\r\n");
  uint8_t http_socket = HTTP_SOCKET;
  uint8_t code = socket(http_socket, Sn_MR_TCP, 10888, 0);
  if(code != http_socket) {
    printf("socket() failed, code = %d\r\n", code);
    return;
  }

  printf("Socket created, connecting...\r\n");
  code = connect(http_socket, addr, 80);
  if(code != SOCK_OK) {
    printf("connect() failed, code = %d\r\n", code);
    close(http_socket);
    return;
  }

  printf("Connected, sending HTTP request...\r\n");
  {
      char req[] = "GET / HTTP/1.0\r\nHost: eax.me\r\n\r\n";
      uint16_t len = sizeof(req) - 1;
      uint8_t* buff = (uint8_t*)&req;
      while(len > 0) {
        printf("Sending %d bytes...\r\n", len);
          int32_t nbytes = send(http_socket, buff, len);
          if(nbytes <= 0) {
            printf("send() failed, %d returned\r\n", nbytes);
            close(http_socket);
            return;
          }
          printf("%d bytes sent!\r\n", nbytes);
          len -= nbytes;
      }
  }

  printf("Request sent. Reading response...\r\n");
  {
      char buff[32];
      for(;;) {
          int32_t nbytes = recv(http_socket, (uint8_t*)&buff, sizeof(buff)-1);
          if(nbytes == SOCKERR_SOCKSTATUS) {
            printf("\r\nConnection closed.\r\n");
            break;
          }

          if(nbytes <= 0) {
            printf("\r\nrecv() failed, %d returned\r\n", nbytes);
            break;
          }

          buff[nbytes] = '\0';
          printf("%s", buff);
      }
  }

  printf("Closing socket.\r\n");
  close(http_socket);

  for (;;) 
  {
    
    
    osDelay(500);
  }
}

void spi_flash_test_task(void const * argument)
{
  SPI_Test();
  for(;;)
  {
    osDelay(500);
    vTaskDelete(NULL);
  }
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread. 
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6)
  {
    g_time_ms++;
  }
  /* USER CODE END Callback 1 */
}

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
