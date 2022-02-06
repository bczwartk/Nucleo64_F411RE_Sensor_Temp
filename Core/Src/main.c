/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * See https://forbot.pl/blog/kurs-stm32l4-termometry-ds18b20-1-wire-uart-id47771
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
// #include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define US_CLOCK_DELAY (0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return 1;
}

void delay_us(uint32_t us)
{
  // TODO: assert: 'us' must be smaller than 2^16, this is max counter period for 16-bit timers
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while ((__HAL_TIM_GET_COUNTER(&htim3)) < us) {  }
}

#if ! US_CLOCK_DELAY
static void set_baudrate(uint32_t baudrate)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = baudrate;
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
}
#endif

HAL_StatusTypeDef wire_reset(void)
{
#if US_CLOCK_DELAY
	  int rc;

	  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	  delay_us(480);
	  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	  delay_us(70);
	  rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
	  delay_us(410);

	  if (rc == 0)
	    return HAL_OK;
	  else
	    return HAL_ERROR;
#else
  uint8_t data_out = 0xF0;
  uint8_t data_in = 0;

  set_baudrate(9600);
  HAL_UART_Transmit(&huart1, &data_out, 1, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart1, &data_in, 1, HAL_MAX_DELAY);
  set_baudrate(115200);

  if (data_in != 0xF0)
    return HAL_OK;
  else
    return HAL_ERROR;
#endif
}

static void write_bit(int value)
{
#if US_CLOCK_DELAY
	  if (value) {
	    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	    delay_us(6);
	    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	    delay_us(64);
	  } else {
	    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	    delay_us(60);
	    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	    delay_us(10);
	  }
#else
  if (value) {
      uint8_t data_out = 0xff;
    HAL_UART_Transmit(&huart1, &data_out, 1, HAL_MAX_DELAY);
  } else {
      uint8_t data_out = 0x0;
    HAL_UART_Transmit(&huart1, &data_out, 1, HAL_MAX_DELAY);
  }
#endif
}

static int read_bit(void)
{
#if US_CLOCK_DELAY
	  int rc;
	  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	  delay_us(6);
	  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	  delay_us(9);
	  rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
	  delay_us(55);
	  return rc;
#else
  uint8_t data_out = 0xFF;
  uint8_t data_in = 0;
  HAL_UART_Transmit(&huart1, &data_out, 1, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart1, &data_in, 1, HAL_MAX_DELAY);
  return data_in & 0x01;
#endif
}

HAL_StatusTypeDef wire_init(void)
{
  return HAL_TIM_Base_Start(&htim3);
}

uint8_t wire_read(void)
{
  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (read_bit())
      value |= 0x80;
  }
  return value;
}

void wire_read_buf(uint8_t * buf, int len)
{
    for (int i = 0; i < len; i++) {
      buf[i] = wire_read();
    }
}

void wire_write(uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    write_bit(byte & 0x01);
    byte >>= 1;
  }
}

static uint8_t byte_crc(uint8_t crc, uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    uint8_t b = crc ^ byte;
    crc >>= 1;
    if (b & 0x01)
      crc ^= 0x8c;
    byte >>= 1;
  }
  return crc;
}

uint8_t wire_crc(const uint8_t* data, int len)
{
  int i;
    uint8_t crc = 0;

    for (i = 0; i < len; i++)
      crc = byte_crc(crc, data[i]);

    return crc;
}


#define DS18B20_ROM_CODE_SIZE		8
#define DS18B20_SCRATCHPAD_SIZE    9
#define DS18B20_READ_ROM           0x33
#define DS18B20_MATCH_ROM          0x55
#define DS18B20_SKIP_ROM           0xCC
#define DS18B20_CONVERT_T          0x44
#define DS18B20_READ_SCRATCHPAD    0xBE

HAL_StatusTypeDef ds18b20_init(void)
{
  return wire_init();
}

HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code)
{
  int i;
  uint8_t crc;

  if (wire_reset() != HAL_OK)
    return HAL_ERROR;

  wire_write(DS18B20_READ_ROM);

  for (i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
    rom_code[i] = wire_read();

  crc = wire_crc(rom_code, DS18B20_ROM_CODE_SIZE - 1);
  if (rom_code[DS18B20_ROM_CODE_SIZE - 1] == crc)
    return HAL_OK;
  else
    return HAL_ERROR;
}

static HAL_StatusTypeDef send_cmd(const uint8_t* rom_code, uint8_t cmd)
{
  int i;

  if (wire_reset() != HAL_OK)
    return HAL_ERROR;

  if (!rom_code) {
    wire_write(DS18B20_SKIP_ROM);
  } else {
    wire_write(DS18B20_MATCH_ROM);
    for (i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
      wire_write(rom_code[i]);
  }
  wire_write(cmd);
  return HAL_OK;
}

HAL_StatusTypeDef ds18b20_start_measure(const uint8_t* rom_code)
{
  return send_cmd(rom_code, DS18B20_CONVERT_T);
}

static HAL_StatusTypeDef ds18b20_read_scratchpad(const uint8_t* rom_code, uint8_t* scratchpad)
{
  int i;
  uint8_t crc;

  if (send_cmd(rom_code, DS18B20_READ_SCRATCHPAD) != HAL_OK)
    return HAL_ERROR;

  for (i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
    scratchpad[i] = wire_read();

  crc = wire_crc(scratchpad, DS18B20_SCRATCHPAD_SIZE - 1);
  if (scratchpad[DS18B20_SCRATCHPAD_SIZE - 1] == crc)
    return HAL_OK;
  else
    return HAL_ERROR;
}

float ds18b20_get_temp(const uint8_t* rom_code)
{
  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  int16_t temp;

  if (ds18b20_read_scratchpad(rom_code, scratchpad) != HAL_OK)
    return 85.0f;

  memcpy(&temp, &scratchpad[0], sizeof(temp));

  return temp / 16.0f;
}

volatile uint32_t push_counter;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_BUTTON_Pin) {
	  push_counter ++;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //printf("Hello!\n");

#if 0
  if (ds18b20_init() != HAL_OK) {
    Error_Handler();
  }

  uint8_t ds1[DS18B20_ROM_CODE_SIZE];

  if (ds18b20_read_address(ds1) != HAL_OK) {
    Error_Handler();
  }
#endif

  // HAL_TIM_Base_Start_IT(&htim3);

  for (int i = 0; i < 10; i++) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(50);
  }

  if (ds18b20_init() != HAL_OK) {
    Error_Handler();
  }

  const uint8_t ds1[] = { 0x28, 0xFF, 0xF2, 0xB5, 0x50, 0x16, 0x03, 0xA2 };
  const uint8_t ds2[] = { 0x28, 0x21, 0x20, 0xD6, 0x07, 0x00, 0x00, 0x15 };

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ds18b20_start_measure(ds1);
	  ds18b20_start_measure(ds2);
	  HAL_Delay(750);

	  float temp = ds18b20_get_temp(ds1);
	  if (temp <= -80.0f) {
	    printf("Sensor error (1)...\n");
	  } else {
	    printf("T1 = %.1f*C\n", temp);
	  }

	  temp = ds18b20_get_temp(ds2);
	  if (temp <= -80.0f) {
	    printf("Sensor error (2)...\n");
	  } else {
	    printf("T2 = %.1f*C\n", temp);
	  }

	  HAL_Delay(5000);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // printf("systick = %lu\n", HAL_GetTick());
	  // printf("counter = %lu\n", push_counter);
#if 0
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		// delay_us(200 * 1000);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		// delay_us(200 * 1000);
#endif
#if 0
		  RTC_TimeTypeDef time;
		  RTC_DateTypeDef date;
		  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
		  printf("Current time: %02d:%02d:%02d\n", time.Hours, time.Minutes, time.Seconds);
#endif
#if 0
		  ds18b20_start_measure(NULL);

		  HAL_Delay(750);

		  float temp = ds18b20_get_temp(NULL);
		  if (temp >= 80.0f)
		    printf("Sensor error...\n");
		  else
		    printf("T1 = %.1f*C\n", temp);
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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

