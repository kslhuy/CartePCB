/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "iim42652.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── IMU chip-select pin (PA4, manually driven as GPIO) ── */
#define IMU_CS_PIN        GPIO_PIN_4
#define IMU_CS_PORT       GPIOA

/* ── SPI3 slave packet format (NAV → COM) ── */
#define NAV_PKT_HEADER    0xAAu
#define NAV_PKT_FOOTER    0x55u
#define NAV_PKT_SIZE      18u
/*  Byte layout:
 *    [0]      0xAA  header
 *    [1]      status  (0x01 = IMU data valid)
 *    [2..3]   accel_x   (big-endian int16)
 *    [4..5]   accel_y
 *    [6..7]   accel_z
 *    [8..9]   gyro_x
 *    [10..11]  gyro_y
 *    [12..13]  gyro_z
 *    [14..15]  temperature
 *    [16]     XOR checksum (bytes 0..15)
 *    [17]     0x55  footer
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* IIM-42652 IMU driver instance */
static IIM42652_Handle_t imu;
static IIM42652_Data_t   imu_data;

/* Double-buffered SPI3 slave TX (ping-pong) */
static uint8_t           spi3_tx_buf[2][NAV_PKT_SIZE];
static volatile uint8_t  spi3_ready_idx = 0;  /* index of latest COMPLETE packet */

/* UART debug buffer */
static char uart_buf[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void NAV_PackSpiBuffer(uint8_t *buf, const IIM42652_Data_t *d, uint8_t status);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Pack IMU data into NAV SPI3 slave TX packet.
 */
static void NAV_PackSpiBuffer(uint8_t *buf, const IIM42652_Data_t *d, uint8_t status)
{
    buf[0]  = NAV_PKT_HEADER;
    buf[1]  = status;
    buf[2]  = (uint8_t)(d->accel_x >> 8);
    buf[3]  = (uint8_t)(d->accel_x);
    buf[4]  = (uint8_t)(d->accel_y >> 8);
    buf[5]  = (uint8_t)(d->accel_y);
    buf[6]  = (uint8_t)(d->accel_z >> 8);
    buf[7]  = (uint8_t)(d->accel_z);
    buf[8]  = (uint8_t)(d->gyro_x >> 8);
    buf[9]  = (uint8_t)(d->gyro_x);
    buf[10] = (uint8_t)(d->gyro_y >> 8);
    buf[11] = (uint8_t)(d->gyro_y);
    buf[12] = (uint8_t)(d->gyro_z >> 8);
    buf[13] = (uint8_t)(d->gyro_z);
    buf[14] = (uint8_t)(d->temperature >> 8);
    buf[15] = (uint8_t)(d->temperature);

    /* XOR checksum over bytes 0..15 */
    uint8_t chk = 0;
    for (int i = 0; i < 16; i++) chk ^= buf[i];
    buf[16] = chk;
    buf[17] = NAV_PKT_FOOTER;
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
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* ── Reconfigure PA4 as GPIO output for manual IMU chip-select ── */
  {
      GPIO_InitTypeDef gpio = {0};
      gpio.Pin   = IMU_CS_PIN;
      gpio.Mode  = GPIO_MODE_OUTPUT_PP;
      gpio.Pull  = GPIO_NOPULL;
      gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      HAL_GPIO_Init(IMU_CS_PORT, &gpio);
      HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET); /* CS idle high */
  }

  /* ── Initialise IIM-42652 IMU on SPI1 ── */
  if (IIM42652_Init(&imu, &hspi1, IMU_CS_PORT, IMU_CS_PIN) != HAL_OK)
  {
      /* IMU init failed (WHO_AM_I mismatch or wiring) — blink PB13 fast */
      HAL_UART_Transmit(&huart4, (uint8_t *)"IMU FAIL\r\n", 10, HAL_MAX_DELAY);
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
          HAL_Delay(100);
      }
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"IMU OK\r\n", 9, HAL_MAX_DELAY);

  /* ── Prepare SPI3 slave double-buffers ── */
  memset(spi3_tx_buf, 0, sizeof(spi3_tx_buf));
  spi3_tx_buf[0][0]  = NAV_PKT_HEADER;
  spi3_tx_buf[0][17] = NAV_PKT_FOOTER;
  spi3_tx_buf[1][0]  = NAV_PKT_HEADER;
  spi3_tx_buf[1][17] = NAV_PKT_FOOTER;

  /* Enable SPI3 interrupt (needed for HAL_SPI_Transmit_IT in slave mode) */
  HAL_NVIC_SetPriority(SPI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(SPI3_IRQn);

  /* ── PA15 as SPI3_NSS (hardware) + EXTI rising edge for TX re-arm ── */
  {
      /* 1) Configure PA15 as AF6 = SPI3_NSS (hardware slave-select).
       *    The SPI peripheral now tracks this pin directly and resets
       *    the shift register / bit counter on every CS deassertion.   */
      GPIO_InitTypeDef gpio = {0};
      gpio.Pin       = GPIO_PIN_15;
      gpio.Mode      = GPIO_MODE_AF_PP;
      gpio.Pull      = GPIO_PULLUP;
      gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
      gpio.Alternate = GPIO_AF6_SPI3;
      HAL_GPIO_Init(GPIOA, &gpio);

      /* 2) Layer EXTI15 on top via registers — the GPIO stays in AF mode
       *    but EXTI still fires because the input path is always active. */
      __HAL_RCC_SYSCFG_CLK_ENABLE();
      SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI15;   /* map to PA        */
      EXTI->RTSR       |=  (1u << 15);                 /* rising edge      */
      EXTI->FTSR       &= ~(1u << 15);                 /* no falling edge  */
      EXTI->IMR        |=  (1u << 15);                 /* unmask           */
      HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }

  /* Arm SPI3 slave with buffer 0 — ready for master (COM/H753) to clock */
  spi3_ready_idx = 0;
  HAL_SPI_Transmit_IT(&hspi3, spi3_tx_buf[0], NAV_PKT_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1) Read IMU sensor data via SPI1 */
    IIM42652_ReadSensorData(&imu, &imu_data);

    /* 2) Pack fresh data into the INACTIVE SPI3 TX buffer, then publish it */
    {
        uint8_t wr = spi3_ready_idx ^ 1;          /* write to other buffer   */
        NAV_PackSpiBuffer(spi3_tx_buf[wr], &imu_data, 0x01);
        spi3_ready_idx = wr;                       /* atomic 8-bit store      */
    }

    /* 3) Heartbeat LED (PB14) + debug UART every 500 ms */
    {
        static uint32_t last_print = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_print >= 500u)
        {
            last_print = now;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

            int len = snprintf(uart_buf, sizeof(uart_buf),
                "AX:%6d AY:%6d AZ:%6d GX:%6d GY:%6d GZ:%6d T:%d\r\n",
                imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                imu_data.gyro_x,  imu_data.gyro_y,  imu_data.gyro_z,
                imu_data.temperature);
            HAL_UART_Transmit(&huart4, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);
        }
    }

    HAL_Delay(1);  /* ~1 kHz loop rate */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_A_NAV_Pin|LED_B_NAV_Pin|LED_C_NAV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_ACC_INT_Pin IMU_GYR_INT_Pin MAG_INT_Pin */
  GPIO_InitStruct.Pin = IMU_ACC_INT_Pin|IMU_GYR_INT_Pin|MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_A_NAV_Pin LED_B_NAV_Pin LED_C_NAV_Pin */
  GPIO_InitStruct.Pin = LED_A_NAV_Pin|LED_B_NAV_Pin|LED_C_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  EXTI callback — PA15 rising edge = CS went HIGH (master done).
 *         Abort any partial SPI3 transfer and re-arm from byte 0 with latest data.
 *         This guarantees frame alignment for every master read.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15)
    {
        /* Abort any ongoing/partial SPI3 slave transfer */
        HAL_SPI_Abort(&hspi3);

        /* Re-arm with the latest complete packet buffer */
        HAL_SPI_Transmit_IT(&hspi3, spi3_tx_buf[spi3_ready_idx], NAV_PKT_SIZE);
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
#ifdef USE_FULL_ASSERT
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
