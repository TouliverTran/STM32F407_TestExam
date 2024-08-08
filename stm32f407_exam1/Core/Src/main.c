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
#include "yahdlc.h"
#include "function.h"
#include "function_test.h"
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
extern uint16_t timeoutReceive = 70;
extern uint16_t delaytimer = 1000;
extern uint8_t newReceive = 0;

yahdlc_control_t control ={0};
uint8_t yahdlc_send_frame[BUF_SIZE] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int uart_printf(const char *fmt, ...);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

extern int debug_printf(const char *fmt, ...);
void debug_sendchar(const char pdata);
void append_char_to_buffer(char *buffer, size_t buffer_size, char c);
void ProcessDataFromUART(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data_rx[2];
uint8_t buf_rx[BUF_SIZE];
uint16_t t = 0;
yahdlc_state_t state;
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
  // debug_printf("HELLO WORLD!!!\r\n");
  HAL_UART_Receive_IT(&huart1, data_rx, 1);
  memset(yahdlc_send_frame, 0, BUF_SIZE);
  tc_add();
  debug_printf("%s\n",yahdlc_send_frame);
  memset(yahdlc_send_frame, 0, BUF_SIZE);
  tc_subtract();
  debug_printf("%s\n",yahdlc_send_frame);
  memset(yahdlc_send_frame, 0, BUF_SIZE);
  tc_multiply();
  debug_printf("%s\n",yahdlc_send_frame);
  memset(yahdlc_send_frame, 0, BUF_SIZE);
  tc_divide();
  debug_printf("%s\n",yahdlc_send_frame);

  yahdlc_set_state(&state);

  control.frame = YAHDLC_FRAME_DATA;
  control.seq_no = 0;

  uint8_t input_data[] = "1234567\r\n";
  int frame_len;

  yahdlc_frame_data(&control, input_data, sizeof(input_data)-1, yahdlc_send_frame, &frame_len);
  for(int i=0; i<frame_len; i++)
  {
    // debug_sendchar(yahdlc_send_frame[i]);
    uart_sendchar(yahdlc_send_frame[i]);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if(delaytimer == 0)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
      delaytimer = 1000;
      // for(int i=0; i<frame_len; i++)
      // {
      //   uart_sendchar(yahdlc_send_frame[i]);
      // }
    }

    ProcessDataFromUART();
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

void append_char_to_buffer(char *buffer, size_t buffer_size, char c) {
    size_t len = strlen(buffer);
    if (len + 1 < buffer_size) {
        buffer[len] = c;
        buffer[len + 1] = '\0';
    } else {
        return;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  timeoutReceive = 70;
  // debug_sendchar(data_rx[0]);
  memcpy(buf_rx+t, data_rx, 1);
  if (++t >= BUF_SIZE) t = 0;
  HAL_UART_Receive_IT(&huart1, data_rx, 1);
}

void ProcessDataFromUART(void)
{
	/* Neu khong co du lieu thi khong xu ly */
	if(newReceive == 0)
		return;
	
	/* Xoa co bao co du lieu */
	newReceive = 0;
	/* Xu ly lenh */
  int ret;
  uint8_t received_data[BUF_SIZE] = {0};
  int received_len = t;
  ret = yahdlc_get_data(&control, buf_rx, t, received_data, &received_len);
  if (ret == 0) {
    if (control.frame == YAHDLC_FRAME_ACK)
    {
      ; // Do nothing
    }
    else if (control.frame == YAHDLC_FRAME_NACK)
    {
      ; // Do nothing
    }
    else if (control.frame == YAHDLC_FRAME_DATA)
    {
      debug_printf("\r\ncontrol field: frame = %d, seq_no = %d\r\n", control.frame, control.seq_no);
      debug_printf("Received data: \r\n");
      for(int i=0; i<received_len; i++)
      {
        debug_sendchar(received_data[i]);
      }
      control.frame = YAHDLC_FRAME_ACK;
      control.seq_no += 1;
      // Generate the ACK frame
      char frame[BUF_SIZE];
      unsigned int frame_len;
      yahdlc_frame_data(&control, NULL, 0, frame, &frame_len);
      for (unsigned int i = 0; i < frame_len; i++) {
        uart_sendchar(frame[i]);
      }

      // Process receive data buffer
      if((strstr((char*)received_data, "tc_")))
      {
        memset(yahdlc_send_frame, 0, BUF_SIZE);
        if((strstr((char*)received_data, "tc_add")))
        {
          tc_add();
        }
        else if((strstr((char*)received_data, "tc_subtract")))
        {
          tc_subtract();
        }
        else if((strstr((char*)received_data, "tc_multiply")))
        {
          tc_multiply();
        }
        else if((strstr((char*)received_data, "tc_divide")))
        {
          tc_divide();
        }

        debug_printf("\n======\n%s\n - length = %d\n",yahdlc_send_frame, strlen(yahdlc_send_frame));

        HAL_Delay(1000);
        control.frame = YAHDLC_FRAME_DATA;
        control.seq_no += 1;
        memset(frame, 0, BUF_SIZE);
        yahdlc_frame_data(&control, yahdlc_send_frame, strlen(yahdlc_send_frame), frame, &frame_len);
        // debug_printf("%s\n - length = %d\n", frame, frame_len);

        for (unsigned int i = 0; i < frame_len; i++) {
          uart_sendchar(frame[i]);
        }
      }
    }
    else
    {
      ; // Do nothing
    }
  } else {
      // debug_printf("Error decoding frame %d\r\n", ret);
      // debug_printf("t = %d buf_rx:\r\n", t);
      // for(int i=0; i<t; i++)
      // {
      //   debug_printf("0x%02x, ", buf_rx[i]);
      // }
      // debug_printf("\r\nreceived_len = %d received_data:\r\n", received_len);
      // for(int i=0; i<received_len; i++)
      // {
      //   debug_printf("0x%02x, ", received_data[i]);
      // }
      yahdlc_control_t control;
      control.frame = YAHDLC_FRAME_NACK;
      control.seq_no = 0;
      // Generate the ACK frame
      char frame[BUF_SIZE];
      unsigned int frame_len;
      yahdlc_frame_data(&control, NULL, 0, frame, &frame_len);
      for (unsigned int i = 0; i < frame_len; i++) {
        uart_sendchar(frame[i]);
      }
  }
	/* Xoa bo dem nhan */
	t = 0;
	memset(buf_rx, 0, BUF_SIZE);
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
