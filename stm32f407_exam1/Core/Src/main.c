/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

#include "unity.h"
#include "minihdlc.h"
#include "yahdlc.h"
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int uart_printf(const char *fmt, ...);

int add(int a, int b);
void test_add(void);
void setUp();
void tearDown();
void send_char(uint8_t data);
void frame_received(const uint8_t *frame_buffer, uint16_t frame_length);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

extern int debug_printf(const char *fmt, ...);
extern void debug_sendchar(const char pdata);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data_rx[2];
uint8_t buf_rx[20];
uint8_t t = 0;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  debug_printf("HELLO WORLD!!!\r\n");
  HAL_UART_Receive_IT(&huart1, data_rx, 1);
  UNITY_BEGIN();
  RUN_TEST(test_add);
  UNITY_END();
  minihdlc_init(send_char, frame_received);
  char string_buffer[256];
  for (int i = 0; i < 10; i++)
  {
      snprintf(string_buffer, sizeof(string_buffer), "i = %d\r\n", i);
      minihdlc_send_frame((const uint8_t *)string_buffer, strlen(string_buffer));
  }




  yahdlc_state_t state;
  yahdlc_set_state(&state);

  yahdlc_control_t control;
  control.frame = YAHDLC_FRAME_DATA;
  control.seq_no = 0;

  uint8_t input_data[] = "1234567890\r\n";
  uint8_t output_frame[256];
  int frame_len;

  frame_len = yahdlc_frame_data(&control, input_data, sizeof(input_data), output_frame, sizeof(output_frame));
  for(int i=0; i<frame_len; i++)
  {
    debug_sendchar(output_frame[i]);
    uart_sendchar(output_frame[i]);
  }


  // int ret;
  // uint8_t received_data[256];
  // int received_len;

  // ret = yahdlc_get_data(&state, output_frame, frame_len, received_data, &received_len);
  // if (ret == 0) {
  //     debug_printf("Received data: %s\n", received_data);
  // } else {
  //     debug_printf("Error decoding frame\n");
  // }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    HAL_Delay(1000);
    for(int i=0; i<frame_len; i++)
    {
      debug_sendchar(output_frame[i]);
      uart_sendchar(output_frame[i]);
    }
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
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int uart_printf(const char *fmt, ...)
{
  char buf[1024];
  int len, i;
  va_list args;
  va_start (args, fmt);

  len = vsprintf(buf, fmt, args);
  for (i=0; i<len; i++)
  {
    HAL_UART_Transmit(&huart1, &buf[i], sizeof(char), 10);
  }
  va_end(args);
  return len;
}

void uart_sendchar(const char pdata)
{
  char temp = pdata;
  HAL_UART_Transmit(&huart1, &temp, sizeof(char), 10);
  // if ('\n' == pdata)
  // {
  //   temp = '\r';
  //   HAL_UART_Transmit(&huart1, &temp, sizeof(char), 10);
  // }
}

int debug_printf(const char *fmt, ...)
{
  char buf[1024];
  int len, i;
  va_list args;
  va_start (args, fmt);

  len = vsprintf(buf, fmt, args);
  for (i=0; i<len; i++)
  {
    HAL_UART_Transmit(&huart2, &buf[i], sizeof(char), 10);
  }
  va_end(args);
  return len;
}

void debug_sendchar(const char pdata)
{
  char temp = pdata;
  HAL_UART_Transmit(&huart2, &temp, sizeof(char), 10);
  // if ('\n' == pdata)
  // {
  //   temp = '\r';
  //   HAL_UART_Transmit(&huart1, &temp, sizeof(char), 10);
  // }
}

int add(int a, int b){
    return a + b;
}

void test_add(void){
    for(int i=0; i<9; i++){
        for(int j=0; j<9; j++){
            TEST_ASSERT_EQUAL_INT((i+j), add(i,j));
        }
    }
}

void setUp(){

}
void tearDown(){

}

void send_char(uint8_t data)
{
    uart_sendchar(data);
}

void frame_received(const uint8_t *frame_buffer, uint16_t frame_length)
{
  debug_printf("\r\nReceived frame: ");
  for (uint16_t i = 0; i < frame_length; i++)
  {
      debug_sendchar(frame_buffer[i]);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  minihdlc_char_receiver(data_rx[0]);
  // debug_sendchar(data_rx[0]);
  // memcpy(buf_rx+t, data_rx, 1);
  // if (++t >= 20) t = 0;
  HAL_UART_Receive_IT(&huart1, data_rx, 1);
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
